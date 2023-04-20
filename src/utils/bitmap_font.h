/*
  bitmap_font.h (PNM).
  for n in `seq 32 126`; do printf "\\$(printf %o $n)"; done | pbmtext -space 0 -lspace 0 -nomargins -builtin fixed >| tmp.pbm
  convert tmp.pbm bitmap_font.h
*/
enum {
        FONT_W = 7,
        FONT_H = 12,
        FONT_COUNT = ('~' - ' ' + 1),
};

static const unsigned char
  font[] =
  {
    0x50, 0x34, 0x0A, 0x36, 0x36, 0x35, 0x20, 0x31, 0x32, 0x0A, 0x00, 0x20, 
    0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 
    0x38, 0x20, 0xE1, 0xC0, 0x8F, 0x87, 0x3E, 0x38, 0x70, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0xC4, 0x06, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0xC0, 0x07, 
    0x00, 0xC0, 0x20, 0x26, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 
    0xA0, 0xA3, 0x86, 0x0E, 0x08, 0x08, 0x40, 0x40, 0x00, 0x00, 0x00, 0x02, 
    0x44, 0xE1, 0x12, 0x21, 0x88, 0x08, 0x22, 0x44, 0x88, 0x00, 0x00, 0x00, 
    0x00, 0x1C, 0x38, 0xE3, 0xE1, 0xCF, 0x9F, 0xBF, 0x1C, 0xEE, 0xF8, 0xF7, 
    0x6E, 0x18, 0xF3, 0x9C, 0xF8, 0x73, 0xE1, 0xEF, 0xFD, 0xFB, 0xF7, 0xEF, 
    0xDD, 0xF0, 0x82, 0x02, 0x04, 0x00, 0x10, 0x01, 0x00, 0x00, 0x40, 0x08, 
    0x00, 0x40, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x00, 0x00, 0x20, 
    0xA0, 0xA4, 0x49, 0x12, 0x08, 0x08, 0x40, 0x40, 0x00, 0x00, 0x00, 0x02, 
    0x44, 0x21, 0x10, 0x22, 0x88, 0x10, 0x04, 0x44, 0x88, 0x00, 0x00, 0x80, 
    0x08, 0x22, 0x44, 0x21, 0x12, 0x24, 0x48, 0x91, 0x22, 0x44, 0x20, 0x22, 
    0x44, 0x0D, 0x99, 0x22, 0x4C, 0x89, 0x12, 0x29, 0x28, 0x91, 0x22, 0x44, 
    0x89, 0x10, 0x82, 0x02, 0x0A, 0x00, 0x10, 0x01, 0x00, 0x00, 0x40, 0x08, 
    0x00, 0x40, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x02, 0x00, 0x00, 0x00, 0x20, 
    0xA3, 0xF4, 0x06, 0x50, 0x10, 0x10, 0x21, 0xF0, 0x80, 0x00, 0x00, 0x04, 
    0x44, 0x21, 0x10, 0x22, 0x88, 0x10, 0x04, 0x44, 0x88, 0x00, 0x01, 0x00, 
    0x04, 0x02, 0x4C, 0x51, 0x12, 0x24, 0x4A, 0x15, 0x22, 0x44, 0x20, 0x22, 
    0x84, 0x0D, 0x99, 0x22, 0x44, 0x89, 0x12, 0x09, 0x28, 0x91, 0x2A, 0x28, 
    0x51, 0x20, 0x81, 0x02, 0x11, 0x00, 0x08, 0xF1, 0xE1, 0xC3, 0xC7, 0x1F, 
    0x1D, 0x58, 0xE1, 0xE2, 0x61, 0x1D, 0x2E, 0x1C, 0xB8, 0x75, 0x71, 0xC7, 
    0xD9, 0xBB, 0xF7, 0xEF, 0xDD, 0xF0, 0x81, 0x02, 0x00, 0x00, 0x00, 0x20, 
    0x01, 0x43, 0x01, 0x88, 0x80, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x04, 
    0x44, 0x20, 0x20, 0xC4, 0x8F, 0x1E, 0x04, 0x38, 0x88, 0x40, 0x82, 0x0F, 
    0x82, 0x04, 0x54, 0x51, 0xE2, 0x04, 0x4E, 0x1C, 0x20, 0x7C, 0x20, 0x23, 
    0x04, 0x0A, 0x95, 0x22, 0x4C, 0x89, 0xE3, 0x81, 0x08, 0x9B, 0x2A, 0x10, 
    0x50, 0x40, 0x81, 0x02, 0x00, 0x00, 0x00, 0x09, 0x12, 0x24, 0x48, 0x88, 
    0x22, 0x64, 0x20, 0x22, 0x81, 0x0A, 0x99, 0x22, 0x44, 0x88, 0x82, 0x22, 
    0x08, 0x91, 0x22, 0x28, 0x89, 0x20, 0x81, 0x02, 0x09, 0x00, 0x00, 0x20, 
    0x01, 0x41, 0x86, 0x1D, 0x00, 0x10, 0x20, 0xA3, 0xE0, 0x0F, 0x80, 0x08, 
    0x44, 0x20, 0x40, 0x28, 0x80, 0x91, 0x04, 0x44, 0x78, 0x00, 0x04, 0x00, 
    0x01, 0x08, 0x54, 0xF9, 0x12, 0x04, 0x4A, 0x14, 0x26, 0x44, 0x20, 0x23, 
    0x84, 0x0A, 0x95, 0x22, 0x78, 0x89, 0x20, 0xE1, 0x08, 0x8A, 0x2A, 0x28, 
    0x20, 0x40, 0x80, 0x82, 0x00, 0x00, 0x00, 0x79, 0x12, 0x04, 0x4F, 0x88, 
    0x22, 0x44, 0x20, 0x23, 0x01, 0x0A, 0x91, 0x22, 0x44, 0x88, 0x81, 0xC2, 
    0x08, 0x91, 0x2A, 0x10, 0x88, 0x41, 0x01, 0x01, 0x15, 0x00, 0x00, 0x20, 
    0x07, 0xE4, 0x49, 0xA2, 0x00, 0x10, 0x20, 0x00, 0x80, 0x00, 0x00, 0x08, 
    0x44, 0x20, 0x80, 0x2F, 0xC0, 0x91, 0x08, 0x44, 0x08, 0x00, 0x02, 0x0F, 
    0x82, 0x08, 0x4E, 0x89, 0x12, 0x24, 0x48, 0x10, 0x22, 0x44, 0x21, 0x22, 
    0x44, 0x4A, 0x93, 0x22, 0x40, 0x89, 0x10, 0x21, 0x08, 0x8A, 0x36, 0x28, 
    0x20, 0x90, 0x80, 0x82, 0x00, 0x00, 0x00, 0x89, 0x12, 0x04, 0x48, 0x08, 
    0x22, 0x44, 0x20, 0x22, 0x81, 0x0A, 0x91, 0x22, 0x44, 0x88, 0x80, 0x22, 
    0x08, 0x8A, 0x2A, 0x28, 0x50, 0x80, 0x81, 0x02, 0x12, 0x00, 0x00, 0x00, 
    0x02, 0x84, 0x42, 0x63, 0x00, 0x10, 0x20, 0x00, 0x82, 0x00, 0x00, 0x10, 
    0x44, 0x21, 0x12, 0x20, 0x90, 0x91, 0x08, 0x44, 0x10, 0x00, 0x81, 0x00, 
    0x04, 0x00, 0x40, 0x89, 0x12, 0x24, 0x48, 0x90, 0x22, 0x44, 0x21, 0x22, 
    0x24, 0x48, 0x93, 0x22, 0x40, 0x89, 0x12, 0x21, 0x0D, 0x8E, 0x14, 0x44, 
    0x21, 0x10, 0x80, 0x42, 0x00, 0x00, 0x00, 0x89, 0x12, 0x24, 0x48, 0x88, 
    0x22, 0x44, 0x20, 0x22, 0x41, 0x0A, 0x91, 0x22, 0x44, 0x88, 0x82, 0x22, 
    0x28, 0x8E, 0x14, 0x44, 0x71, 0x10, 0x81, 0x02, 0x00, 0x00, 0x00, 0x20, 
    0x02, 0x83, 0x81, 0x9C, 0x80, 0x08, 0x40, 0x00, 0x02, 0x00, 0x08, 0x10, 
    0x38, 0xF9, 0xF1, 0xC1, 0xCF, 0x0E, 0x08, 0x38, 0xE0, 0x40, 0x80, 0x80, 
    0x08, 0x08, 0x39, 0xDF, 0xE1, 0xCF, 0x9F, 0xBC, 0x1C, 0xEE, 0xF8, 0xC7, 
    0x3F, 0xDD, 0xF9, 0x1C, 0xF0, 0x73, 0x9B, 0xC7, 0xC7, 0x04, 0x14, 0xEE, 
    0x71, 0xF0, 0x80, 0x42, 0x00, 0x00, 0x00, 0x76, 0xE1, 0xC3, 0xA7, 0x1E, 
    0x1E, 0xEE, 0xF8, 0x26, 0x37, 0xFA, 0xF9, 0x9C, 0x78, 0x79, 0xE1, 0xC1, 
    0xC7, 0x44, 0x14, 0xC6, 0x21, 0xF0, 0x81, 0x02, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x40, 0x00, 0x04, 0x00, 0x00, 0x20, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x80, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x02, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x40, 0x08, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x81, 0x02, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0xC0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x02, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x40, 0x08, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x41, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x1C, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1C, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00
  };