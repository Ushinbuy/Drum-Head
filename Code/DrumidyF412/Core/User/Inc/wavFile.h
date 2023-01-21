#ifndef USER_INC_WAVFILE_H_
#define USER_INC_WAVFILE_H_

typedef struct WAVE_FormatTypeDef
{
  uint32_t   ChunkID;       /* 0 	0x00 */
  uint32_t   FileSize;      /* 4 	0x04 */
  uint32_t   FileFormat;    /* 8 	0x08 */
  uint32_t   SubChunk1ID;   /* 12 	0x0C */
  uint32_t   SubChunk1Size; /* 16	0x10 */
  uint16_t   AudioFormat;   /* 20 	0x14 */
  uint16_t   NbrChannels;   /* 22 	0x16 */
  uint32_t   SampleRate;    /* 24 	0x18 */

  uint32_t   ByteRate;      /* 28 	0x1C */
  uint16_t   BlockAlign;    /* 32 	0x20 */
  uint16_t   BitPerSample;  /* 34 	0x22 */
  uint32_t   SubChunk2ID;   /* 36 	0x24 */
  uint32_t   SubChunk2Size; /* 40 	0x28 */

}WAVE_FormatTypeDef;

#endif /* USER_INC_WAVFILE_H_ */
