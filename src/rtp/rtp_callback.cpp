/*
 * FILE:     rtp_callback.c
 * AUTHOR:   Colin Perkins <csp@csperkins.org>
 *           Ladan Gharai  <ladan@isi.edu>
 *           Martin Benes     <martinbenesh@gmail.com>
 *           Lukas Hejtmanek  <xhejtman@ics.muni.cz>
 *           Petr Holub       <hopet@ics.muni.cz>
 *           Milos Liska      <xliska@fi.muni.cz>
 *           Jiri Matela      <matela@ics.muni.cz>
 *           Dalibor Matura   <255899@mail.muni.cz>
 *           Martin Pulec     <martin.pulec@cesnet.cz>
 *           Ian Wesley-Smith <iwsmith@cct.lsu.edu>
 *
 * Copyright (c) 2005-2019 CESNET z.s.p.o.
 * Copyright (c) 2001-2003 University of Southern California
 * Copyright (c) 2003-2004 University of Glasgow
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 * 
 *      This product includes software developed by the University of Southern
 *      California Information Sciences Institute. This product also includes
 *      software developed by CESNET z.s.p.o.
 * 
 * 4. Neither the name of the University, Institute, CESNET nor the names of
 *    its contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Revision: 1.3.2.2 $
 * $Date: 2010/01/30 20:07:35 $
 *
 */

#include "config.h"
#include "config_unix.h"
#include "config_win32.h"

#include <inttypes.h>
#include <stdint.h>

#include "debug.h"
#include "ntp.h"
#include "participant_db.hpp"
#include "pdb.h"
#include "rtp/rtp.h"
#include "rtp/pbuf.h"
#include "rtp/rtp_callback.h"
#include "tfrc.h"
#include "tv.h"
#include "video_display.h"

extern char *frame;

char hdr_buf[100];
struct msghdr msg;
struct iovec iov[10];

extern uint32_t RTT;

static void process_rr(struct rtp *session, rtp_event * e)
{
        float fract_lost, tmp;
        uint32_t ntp_sec, ntp_frac, now;
        rtcp_rr *r = (rtcp_rr *) e->data;

        if (e->ssrc == rtp_my_ssrc(session)) {
                /* Filter out loopback reports */
                return;
        }

        if (r->ssrc == rtp_my_ssrc(session)) {
                /* Received a reception quality report for data we are sending.  */
                /*                                                               */
                /* Check for excessive congestion: exit if it occurs, to protect */
                /* other users of the network.                                   */
                fract_lost = (r->fract_lost * 100.0) / 256.0;   /* percentage lost packets */
                if (fract_lost > 20) {
                        printf("Receiver 0x%08x reports excessive congestion\n",
                               e->ssrc);
                }

                /* Compute network round-trip time:                              */
                if (r->lsr != 0) {
                        ntp64_time(&ntp_sec, &ntp_frac);
                        now = ntp64_to_ntp32(ntp_sec, ntp_frac);

                        if ((now < r->lsr) || (now - r->lsr < r->dlsr)) {
                                /* Packet arrived before it was sent, ignore. This represents */
                                /* a bug in the remote: either our timestamp was incorrectly  */
                                /* echoed back to us, or the remote miscalculated in time for */
                                /* which it held the packet.                                  */
                                debug_msg("Bogus RTT from 0x%08x ignored\n",
                                          e->ssrc);
                        } else {
                                tmp = ((float)(now - r->lsr - r->dlsr)) / 65536.0;      /* RTT in seconds */
                                RTT = tmp * 1000000;    /* RTT in usec */
                        }
                        //debug_msg("  RTT=%d usec\n", RTT);
                }

                int packet_count = 1;
                const rtcp_rr *last_rr = rtp_get_rr(session, e->ssrc, r->ssrc);
                if(last_rr){
                    packet_count = (r->last_seq - last_rr->last_seq);
                }
                if(packet_count < 1) packet_count = 1;

                log_msg(LOG_LEVEL_INFO, "Receiver of 0x%08x reports RTT=%d usec, loss %.2f%% (out of %d packets)\n",
                        r->ssrc, RTT, fract_lost, packet_count);
        }
}

static void
process_sdes(struct pdb *participants, uint32_t ssrc, rtcp_sdes_item * d)
{
        struct pdb_e *e;
        char *sdes_item;

        e = pdb_get(participants, ssrc);
        if (e == NULL) {
                /* We got an SDES packet for a participant we haven't previously seen. */
                /* This can happen, for example, in a partitioned multicast group. Log */
                /* the event, and add the new participant to the database.             */
                debug_msg
                    ("Added previously unseen participant 0x%08" PRIx32 " due to receipt of SDES\n",
                     ssrc);
                pdb_add(participants, ssrc);
                e = pdb_get(participants, ssrc);
        }

        sdes_item = (char*) calloc(d->length + 1, 1);
        strncpy(sdes_item, d->data, d->length);

        switch (d->type) {
        case RTCP_SDES_END:
                free(sdes_item);
                /* This is the end of the SDES list of a packet. */
                /* Nothing for us to deal with.                  */
                break;
        case RTCP_SDES_CNAME:
                if (e->sdes_cname != NULL)
                        free(e->sdes_cname);
                e->sdes_cname = sdes_item;
                break;
        case RTCP_SDES_NAME:
                if (e->sdes_name != NULL)
                        free(e->sdes_name);
                e->sdes_name = sdes_item;
                break;
        case RTCP_SDES_EMAIL:
                if (e->sdes_email != NULL)
                        free(e->sdes_email);
                e->sdes_email = sdes_item;
                break;
        case RTCP_SDES_PHONE:
                if (e->sdes_phone != NULL)
                        free(e->sdes_phone);
                e->sdes_phone = sdes_item;
                break;
        case RTCP_SDES_LOC:
                if (e->sdes_loc != NULL)
                        free(e->sdes_loc);
                e->sdes_loc = sdes_item;
                break;
        case RTCP_SDES_TOOL:
                if (e->sdes_tool != NULL)
                        free(e->sdes_tool);
                e->sdes_tool = sdes_item;
                break;
        case RTCP_SDES_NOTE:
                if (e->sdes_note != NULL)
                        free(e->sdes_note);
                e->sdes_note = sdes_item;
                break;
        case RTCP_SDES_PRIV:
                free(sdes_item);
                break;          /* Ignore private extensions */
        default:
                free(sdes_item);
                debug_msg
                    ("Ignored unknown SDES item (type=0x%02" PRIx32 ") from 0x%08" PRIx32"\n",
                     d->type, ssrc);
        }
}

/**
 * @brief The callback used to process RTP packets once they arrive. This requires a callback type so we can
 *        properly type the participant database retrieved from the RTP Session user data. In an ideal world
 *        this function would be templated, but unfortunately it's used in C contexts, so this is not possible
*/
void rtp_packet_receive_callback(rtp* session, rtp_event* e, CallbackType callbackType) {
    switch(e->type) {
        case RX_RTP:
            {
                // The event owns the copy of the packet that we intend to hand 
                // ownership of into the playout buffer.
                auto packet = (rtp_packet *) e->data;
                switch(callbackType) {
                    case CALLBACK_VIDEO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<vcodec_state>*>(rtp_get_userdata(session));
                            if(auto participantRef = participants->getParticipant(packet->ssrc)) {
                                // If the optional resolves, then extract the reference held within
                                Participant<vcodec_state>& participant = participantRef->get();
                                // Update the TFRC state
                                tfrc_recv_data(participant.getTfrc(), get_time_in_ns(), packet->seq, packet->data_len + 40);
                                // If the packet holds any data insert it into the participant playout buffer
                                if(packet->data_len > 0) {
                                    participant.getPlayoutBuffer()->insert(packet);
                                }
                            }
                            else {
                                LOG(LOG_LEVEL_WARNING) << "Received packet with no participant - SSRC: " << packet->ssrc << "\n";
                            }
                        }
                        break;
                    // Other than the underlying type of the participant this is identical to the above. Unfortunately it is a consequence
                    // of this API being used in C contexts that requires code written like this.
                    case CALLBACK_AUDIO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<pbuf_audio_data>*>(rtp_get_userdata(session));
                            if(auto participantRef = participants->getParticipant(packet->ssrc)) {
                                // If the optional resolves, then extract the reference held within
                                Participant<pbuf_audio_data>& participant = participantRef->get();
                                // Update the TFRC state
                                tfrc_recv_data(participant.getTfrc(), get_time_in_ns(), packet->seq, packet->data_len + 40);
                                // If the packet holds any data insert it into the participant playout buffer
                                if(packet->data_len > 0) {
                                    participant.getPlayoutBuffer()->insert(packet);
                                }
                            }
                            else {
                                LOG(LOG_LEVEL_WARNING) << "Received packet with no participant - SSRC: " << packet->ssrc << "\n";
                            }
                        }
                        break;
                    // This is for older deprecated instances where this call back is being used where the newer API has not yet
                    // been adopted.
                    case CALLBACK_DEPRECATED: 
                        {
                            struct pdb *participants = (struct pdb *)rtp_get_userdata(session);
                            struct pdb_e *state = pdb_get(participants, e->ssrc);
                            tfrc_recv_data(state->tfrc_state, get_time_in_ns(), packet->seq, packet->data_len + 40);
                            if (packet->data_len > 0) {   /* Only process packets that contain data... */
                                pbuf_insert(state->playout_buffer, packet);
                            }
                        }
                        break;
                    default:
                        {
                            LOG(LOG_LEVEL_DEBUG) << "Got unknown callback type: " << callbackType << "\n";
                        }
                }
            }
            break;
        case RX_RR:
            {
                process_rr(session, e);
            }
            break;
        case RX_SDES:
            {
                switch(callbackType) {
                    case CALLBACK_VIDEO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<vcodec_state>*>(rtp_get_userdata(session));
                            if(auto participantRef = participants->getParticipant(e->ssrc)) {
                                participantRef->get().processSdes((rtcp_sdes_item*) e->data);
                            }
                            else {
                                LOG(LOG_LEVEL_WARNING) << "Received event with no participant - SSRC: " << e->ssrc << "\n";
                            }
                        }
                        break;
                    // Other than the underlying type of the participant this is identical to the above. Unfortunately it is a consequence
                    // of this API being used in C contexts that requires code written like this.
                    case CALLBACK_AUDIO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<pbuf_audio_data>*>(rtp_get_userdata(session));
                            if(auto participantRef = participants->getParticipant(e->ssrc)) {
                                participantRef->get().processSdes((rtcp_sdes_item*) e->data);
                            }
                            else {
                                LOG(LOG_LEVEL_WARNING) << "Received event with no participant - SSRC: " << e->ssrc << "\n";
                            }
                        }
                        break;
                    // This is for older deprecated instances where this call back is being used where the newer API has not yet
                    // been adopted.
                    case CALLBACK_DEPRECATED: 
                        {
                            struct pdb *participants = (struct pdb *)rtp_get_userdata(session);
                            process_sdes(participants, e->ssrc, (rtcp_sdes_item *) e->data);
                        }
                        break;
                    default:
                        {
                            LOG(LOG_LEVEL_DEBUG) << "Got unknown callback type: " << callbackType << "\n";
                        }
                }
            }
            break;
        case RX_APP:
            {
                auto packet_app = (rtcp_app *) e->data;
                if (strncmp(packet_app->name, "RTT_", 4) == 0) {
                    assert(packet_app->length == 3);
                    assert(packet_app->subtype == 0);
                }
            }
            break;
        case SOURCE_DELETED:
            {
                switch(callbackType) {
                    case CALLBACK_VIDEO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<vcodec_state>*>(rtp_get_userdata(session));
                            participants->removeParticipant(e->ssrc);
                        }
                        break;
                    // Other than the underlying type of the participant this is identical to the above. Unfortunately it is a consequence
                    // of this API being used in C contexts that requires code written like this.
                    case CALLBACK_AUDIO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<pbuf_audio_data>*>(rtp_get_userdata(session));
                            participants->removeParticipant(e->ssrc);
                        }
                        break;
                    // This is for older deprecated instances where this call back is being used where the newer API has not yet
                    // been adopted.
                    case CALLBACK_DEPRECATED: 
                        {
                            struct pdb *participants = (struct pdb *)rtp_get_userdata(session);
                            struct pdb_e *pdb_item = NULL;
                            if(pdb_remove(participants, e->ssrc, &pdb_item) == 0) {
                                pdb_destroy_item(pdb_item);
                            }
                        }
                        break;
                    default:
                        {
                            LOG(LOG_LEVEL_DEBUG) << "Got unknown callback type: " << callbackType << "\n";
                        }
                }
            }
            break;
        case SOURCE_CREATED:
            {
                switch(callbackType) {
                    case CALLBACK_VIDEO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<vcodec_state>*>(rtp_get_userdata(session));
                            participants->addParticipant(e->ssrc);
                        }
                        break;
                    // Other than the underlying type of the participant this is identical to the above. Unfortunately it is a consequence
                    // of this API being used in C contexts that requires code written like this.
                    case CALLBACK_AUDIO:
                        {
                            auto participants = reinterpret_cast<ParticipantDB<pbuf_audio_data>*>(rtp_get_userdata(session));
                            participants->addParticipant(e->ssrc);
                        }
                        break;
                    // This is for older deprecated instances where this call back is being used where the newer API has not yet
                    // been adopted.
                    case CALLBACK_DEPRECATED: 
                        {
                            struct pdb *participants = (struct pdb *)rtp_get_userdata(session);
                            pdb_add(participants, e->ssrc);
                        }
                        break;
                    default:
                        {
                            LOG(LOG_LEVEL_DEBUG) << "Got unknown callback type: " << callbackType << "\n";
                        }
                }
            }
            break;
        case RX_TFRC_RX:
        case RX_RTCP_START:
        case RX_RTCP_FINISH:
        case RX_BYE:
        case RX_RR_EMPTY:
        case RR_TIMEOUT:
        case RX_SR:
            break;
        default:
            {
                LOG(LOG_LEVEL_DEBUG) << "Unknown RTP event (type=" << e->type << ")\n";
            }
        
    }
}

/**
 * @brief A wrapper function that can be used by C files safely
*/
void rtp_recv_callback(struct rtp* session, rtp_event* e)
{
        rtp_packet_receive_callback(session, e, CallbackType::CALLBACK_DEPRECATED);
}

