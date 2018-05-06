#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <unordered_map>
#include <vector>
#include <string>

#define BLOCK_SIDE 8
#define BLOCK_SIZE 64
#define MAX_Q_TABLE 64
#define N_COMPONENTS 3
#define MAX_CODE_LENGTH 16
#define N_TYPES 2
#define N_DECODE_TREES 2 
#define DC 0 
#define AC 1 

using namespace std;

const unsigned char MarkerStart = 0xff;
const unsigned char MarkerSOI = 0xd8;
const unsigned char MarkerAPP0 = 0xe0;
const unsigned char MarkerDQT = 0xdb;
const unsigned char MarkerSOF = 0xc0;
const unsigned char MarkerDHT = 0xc4;
const unsigned char MarkerDRI = 0xdd;
const unsigned char MarkerSOS = 0xda;
const unsigned char MarkerEOI = 0xd9;

const int zz_order[64] = {0,
    1, 8,
    16, 9, 2,
    3, 10, 17, 24,
    32, 25, 18, 11, 4,
    5, 12, 19, 26, 33, 40,
    48, 41, 34, 27, 20, 13, 6,
    7, 14, 21, 28, 35, 42, 49, 56,
    57, 50, 43, 36, 29, 22, 15,
    23, 30, 37, 44, 51, 58,
    59, 52, 45, 38, 31,
    39, 46, 53, 60,
    61, 54, 47,
    55, 62,
    63};

typedef struct JpegImage {
    int unit;
    int density[2];
    int thumbnail[2];
    int n_q_tables;
    int q_table[MAX_Q_TABLE][BLOCK_SIZE];
    int height, width;
    int subsampling[N_COMPONENTS][2];
    int Hmax, Vmax;
    int q_table_id[N_COMPONENTS];
    // low 16 bits are code, high 16 bits are length
    unordered_map<int, int> huffman_tables[N_TYPES][N_DECODE_TREES];
    int rstn;
    int h_table_id[N_TYPES][N_COMPONENTS];
    int** blocks[N_COMPONENTS];
    int n_rows_mcu, n_cols_mcu;
} JpegImage;

typedef struct BmpImage {
    //rgb value
    // order: B, G, R
    unsigned char* pixel;
    int width, height;
    int pad_width, pad_height;
} BmpImage;

typedef struct BitReader {
    unsigned char buffer[2];
    int remain;
    int eof;
    BitReader() {
        remain = 0;
        eof = 0;
    };
    int read_bit(FILE *fp) {
        int ret;
        int stop = 0;
        if (remain == 0) {
            fread(buffer, 1, 1, fp);
            remain += 8;
            // special indicator
            while (buffer[0] == 0xff && !stop) {
                fread(buffer + 1, 1, 1, fp);
                switch (buffer[1]) {
                    // do nothing, ignore 00
                    case 0x00: 
                        stop = 1;
                        break;
                    case 0xd0: case 0xd1: case 0xd2: case 0xd3: 
                    case 0xd4: case 0xd5: case 0xd6: case 0xd7:
                        fread(buffer, 1, 1, fp);
                        break;
                    case 0xd9: 
                        eof = 1;
                        stop = 1;
                        break;
                    case 0xff: 
                        break;
                    default: 
                        buffer[0] = buffer[1];
                        stop = 1;
                        break;
                }
            }
        }
        ret = ((buffer[0] >> (remain - 1)) & 1);
        remain--;
        return ret;
    }
    int is_eof(void) {
        return eof;
    }
} BitReader;

void readBytes(FILE* fp, auto* buffer, size_t n) {
    fread(buffer, 1, n, fp);
    return;
}

int ConcatBytes(auto high, auto low) {
    return (int)((high << 8) | low);
}

int ConcatInt(int high, int low) {
    return ((high << 16) | low);
}

int clip(int val, int min, int max) {
    if (val <= min)
        return min;
    else if (val >= max)
        return max;
    else
        return val;
}

void readAPP0(FILE* fp, JpegImage* ptr) {
    unsigned char buffer[2];
    int length = 0;
    readBytes(fp, buffer, 2);
    length = ConcatBytes(buffer[0], buffer[1]);
    // identifier & version
    fseek(fp, 7, SEEK_CUR);
    // unit
    readBytes(fp, buffer, 1);
    ptr->unit = (int)buffer[0];
    // density
    readBytes(fp, buffer, 2);
    ptr->density[0] = ConcatBytes(buffer[0], buffer[1]);
    readBytes(fp, buffer, 2);
    ptr->density[1] = ConcatBytes(buffer[0], buffer[1]);
    // thumbnail
    readBytes(fp, buffer, 2);
    ptr->thumbnail[0] = (int)buffer[0];
    ptr->thumbnail[1] = (int)buffer[1];
    // skip remaining
    fseek(fp, length - 16, SEEK_CUR);
    return;
}
void toZigzagOrder(int* table, auto* buffer) {
    for (int i = 0; i < BLOCK_SIZE; i++)
        table[zz_order[i]] = (int)buffer[i];
    return;
}

/*DEPRECATE
void toZigzagOrder(int* table, auto* buffer) {
    int buffer_index = 0;
    int i = 0, j = 0, di = -1, dj = 1;
    for (int diag = 0; diag < BLOCK_SIDE + BLOCK_SIDE -1; diag++) {
        while (i >= 0 && j >= 0 && i < BLOCK_SIDE && j < BLOCK_SIDE) {
            table[i * BLOCK_SIDE + j] = (int)buffer[buffer_index];
            buffer_index++;
            i += di; j+= dj;
        }
        i -= di; j -= dj;
        di *= -1; dj *= -1;
        if ((i == 0 || i == BLOCK_SIDE - 1) && j < BLOCK_SIDE - 1)
            j++;
        else
            i++;
    }
    return;
}
void Fill_Q_table(int* q_table, unsigned char* buffer) {
    int buffer_index = 0;
    int i = 0, j = 0, di = -1, dj = 1;
    for (int diag = 0; diag < BLOCK_SIDE + BLOCK_SIDE -1; diag++) {
        while (i >= 0 && j >= 0 && i < BLOCK_SIDE && j < BLOCK_SIDE) {
            //printf("%d %d\n", i, j);
            q_table[i * BLOCK_SIDE + j] = (int)buffer[buffer_index];
            buffer_index++;
            i += di; j+= dj;
        }
        i -= di; j -= dj;
        di *= -1; dj *= -1;
        if ((i == 0 || i == BLOCK_SIDE - 1) && j < BLOCK_SIDE - 1)
            j++;
        else
            i++;
    }
    return;
}
*/
void readDQT(FILE* fp, JpegImage* ptr) {
    unsigned char buffer[1 + BLOCK_SIZE];
    int length = 0;
    int n_tables = 0;
    int precision = 0, id = 0;
    readBytes(fp, buffer, 2);
    length = ConcatBytes(buffer[0], buffer[1]);
    // calculate number of tables
    n_tables = (length - 2) / (BLOCK_SIZE + 1);
    for (int i = 0; i < n_tables; i++) {
        // read precision
        readBytes(fp, buffer, BLOCK_SIZE + 1);
        precision = (int)(buffer[0] >> 4) + 1;
        id = (int)(buffer[0] & 0x0f);
        ptr->n_q_tables = id + 1;
        toZigzagOrder(ptr->q_table[id], buffer + 1);
        //Fill_Q_table(ptr->q_table[id], buffer + 1);
    }
    return;
}

void readSOF(FILE* fp, JpegImage* ptr) {
    unsigned char buffer[4];
    int length = 0;
    int precision = 0;
    int n_components = 0;
    readBytes(fp, buffer, 2);
    length = ConcatBytes(buffer[0], buffer[1]);
    readBytes(fp, buffer, 1);
    precision = (int)buffer[0];
    readBytes(fp, buffer, 2);
    ptr->height = ConcatBytes(buffer[0], buffer[1]);
    readBytes(fp, buffer, 2);
    ptr->width = ConcatBytes(buffer[0], buffer[1]);
    // color division must be 3
    readBytes(fp, buffer, 1);
    n_components = (int)buffer[0];
    if (n_components != N_COMPONENTS)
        printf("n_components not matching\n");

    ptr->Hmax = 0; ptr->Vmax = 0;
    for (int i = 0; i < n_components; i++) {
        readBytes(fp, buffer, 3);
        if ((int)buffer[0] != i + 1) 
            printf("frame id not matching\n");
        ptr->subsampling[i][0] = (int)(buffer[1] >> 4);
        ptr->subsampling[i][1] = (int)(buffer[1] & 0xf);
        // calculate HMax and VMax
        if (ptr->subsampling[i][0] > ptr->Hmax)
            ptr->Hmax = ptr->subsampling[i][0];
        if (ptr->subsampling[i][1] > ptr->Vmax)
            ptr->Vmax = ptr->subsampling[i][1];
        ptr->q_table_id[i] = (int)buffer[2];
    }
    return;
}

void readDHT(FILE* fp, JpegImage* ptr) {
    unsigned char buffer[MAX_CODE_LENGTH + 1];
    int length, bytes_left;
    int type, table_id;
    int num_leaves[MAX_CODE_LENGTH] = {0}, max_length = 0;
    int code;
    readBytes(fp, buffer, 2);
    length = ConcatBytes(buffer[0], buffer[1]);
    bytes_left = length - 2;
    while (bytes_left > 0) {
        readBytes(fp, buffer, MAX_CODE_LENGTH + 1);
        bytes_left -= MAX_CODE_LENGTH + 1;
        if (bytes_left < 0) 
            printf("bytes_left error\n");
        type = (int)(buffer[0] >> 4);
        if (type >= N_TYPES) 
            printf("type error\n");
        table_id = buffer[0] & 0xf;
        if (table_id >= N_DECODE_TREES)
            printf("huffman table id error\n");
        auto& map = ptr->huffman_tables[type][table_id];
        for (int i = 0; i < MAX_CODE_LENGTH; i++) {
            num_leaves[i] = (int)buffer[i + 1];
            if (num_leaves[i] > 0)
                max_length = i + 1;
        }
        code = 0;
        for (int len = 1; len <= max_length; len++) {
            for (int i = 0; i < num_leaves[len - 1]; i++) {
                readBytes(fp, buffer, 1);
                bytes_left--;
                map[ConcatInt(len, code)] = (int)buffer[0];
                code++;
            }
            code = code << 1;
        }
    }
    return;
}


void writeBMP(BmpImage* ptr, char* output_path) {
	FILE* fp = fopen(output_path, "wb");
	unsigned char header[54] = {
		0x42, 0x4d, 0, 0, 0, 0, 0, 0, 0, 0,
		54, 0, 0, 0, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0
	};
	int width = ptr->width;
    int height = ptr->height;
	long file_size = (long)width * (long)height * 3 + 54;

	header[2] = (unsigned char)(file_size &0x000000ff);
	header[3] = (file_size >> 8) & 0x000000ff;
	header[4] = (file_size >> 16) & 0x000000ff;
	header[5] = (file_size >> 24) & 0x000000ff;

	header[18] = width & 0x000000ff;
	header[19] = (width >> 8) &0x000000ff;
	header[20] = (width >> 16) &0x000000ff;
	header[21] = (width >> 24) &0x000000ff;

	header[22] = height &0x000000ff;
	header[23] = (height >> 8) &0x000000ff;
	header[24] = (height >> 16) &0x000000ff;
	header[25] = (height >> 24) &0x000000ff;

	fwrite(header, sizeof(unsigned char), 54, fp);
	fwrite(ptr->pixel, sizeof(unsigned char), (size_t)(long)width * height * 3, fp);
	fclose(fp);
    return;
}

void readDRI(FILE* fp, JpegImage* ptr) {
	unsigned char buffer[2];
	fseek(fp, 4, SEEK_CUR);
	readBytes(fp, buffer, 2);
	ptr->rstn = ConcatBytes(buffer[0], buffer[1]);
	return;
}

void readSOS(FILE* fp, JpegImage* ptr) {
	unsigned char buffer[3];
	int length, n_components;
	readBytes(fp, buffer, 2);
	length = ConcatBytes(buffer[0], buffer[1]);
	readBytes(fp, buffer, 1);
	n_components = (int)buffer[0];
	if (n_components != N_COMPONENTS)
		printf("n_components not matching\n");
	for (int i = 0; i < N_COMPONENTS; i++) {
		readBytes(fp, buffer, 2);
		int color_id = (int)buffer[0] - 1;
		ptr->h_table_id[DC][color_id] = (int)(buffer[1] >> 4);
		ptr->h_table_id[AC][color_id] = (int)(buffer[1] & 0xf);
	}
	readBytes(fp, buffer, 3);
	return;
}

int translate(int weight, BitReader& BR, FILE* fp) {
	int value = 0, translation = 0;
	int bit = 0, is_negative = 0;
	for (int len = 0; len < weight; len++) {
		bit = BR.read_bit(fp);
		if (BR.is_eof())
			break;
		value = (value << 1) | bit;
		if (len == 0)
			is_negative = ((bit == 0)? 1 : 0); 
	}
	if (is_negative) 
		translation = -(value^((1 << weight) - 1));
	else
		translation = value;
	return translation;
}

void decodeMCU(FILE* fp, auto& dc_table, auto& ac_table, JpegImage* ptr, BitReader& BR, int* table) {
	unsigned char buffer[2];
	int count = 0, terminate = 0;
	while (count < BLOCK_SIZE && !terminate) {
		int length = 0, code = 0, find = 0, weight = 0, translation = 0;
		int n_zeros = 0, ac_value = 0;
		auto& huffman_table = (count == 0)? dc_table: ac_table;
		int bit = 0;
		while (!find) {
			bit = BR.read_bit(fp);
			if (BR.is_eof()) {
				return;
			}
			code = (code << 1) | bit;
			length++;
			auto got = huffman_table.find(ConcatInt(length, code));
			// code found
			if (got != huffman_table.end()) {
				weight = got->second;
				find = 1; 
			}
		}
		// AC terminate code
		if (count > 0 && weight == 0) {
			terminate = 1;
			break;
		}
		// DC 
		if (count == 0) {
			// read weight bits of data
			translation = translate(weight, BR, fp);
			if (BR.is_eof()) {
				return;
			}
			table[count] = translation;
			count++;
		} else {
			// high 4 bits are the running zero, low 4 bits are the value
			n_zeros = weight >> 4;
			weight = weight & 0xf;
			translation = translate(weight, BR, fp);
			if (BR.is_eof()) {
				return;
			}
			// fill in AC value
			count += n_zeros;
			table[count] = translation;
			count++;
		}
	}
	return;
}

int readMCU(FILE* fp, JpegImage* ptr) {
	unsigned char buffer[BLOCK_SIZE];
	BitReader BR;
	int row_mcu = BLOCK_SIDE * ptr->Vmax;
	int col_mcu = BLOCK_SIDE * ptr->Hmax;
	ptr->n_rows_mcu = ceil((float)ptr->height / row_mcu);
	ptr->n_cols_mcu = ceil((float)ptr->width / col_mcu);
	int n_mcu = ptr->n_rows_mcu * ptr->n_cols_mcu;; 
	for (int c = 0; c < N_COMPONENTS; c++) {
		int n_tables = n_mcu * ptr->subsampling[c][0] * ptr->subsampling[c][1];
		ptr->blocks[c] = new int* [n_tables];
		for (int t = 0; t < n_tables; t++) { 
			ptr->blocks[c][t] = new int [BLOCK_SIZE](); 
		}
	}
	int prev_dc[N_COMPONENTS] = {0};
	for (int mcu = 0; mcu < n_mcu; mcu++) {
		for (int c = 0; c < N_COMPONENTS; c++) {
			int dc_table_id = ptr->h_table_id[DC][c];
			int ac_table_id = ptr->h_table_id[AC][c];
			auto& dc_table = ptr->huffman_tables[DC][dc_table_id];
			auto& ac_table = ptr->huffman_tables[AC][ac_table_id];
			int n_tables = ptr->subsampling[c][0] * ptr->subsampling[c][1];
			for (int t = 0; t < n_tables; t++) {
				int buffer_table[BLOCK_SIZE] = {0};
				decodeMCU(fp, dc_table, ac_table, ptr, BR, buffer_table);
				buffer_table[0] += prev_dc[c];
				prev_dc[c] = buffer_table[0];
				toZigzagOrder(ptr->blocks[c][mcu * n_tables + t], buffer_table);
				if (BR.is_eof()) {
					return 1;
				}
			} 
		}
	}
	return 0;
}

void dequantize(JpegImage* ptr) {
	int n_mcu = ptr->n_rows_mcu * ptr->n_cols_mcu;
	for (int c = 0; c < N_COMPONENTS; c++) {
		int n_tables = n_mcu * ptr->subsampling[c][0] * ptr->subsampling[c][1];
		int* q_table = ptr->q_table[ptr->q_table_id[c]]; 
		for (int t = 0; t < n_tables; t++) {
			for (int i = 0; i < BLOCK_SIZE; i++) {
				ptr->blocks[c][t][i] *= q_table[i];
			}
		}
	}
	return;
}

inline void IDCT_block(int* block, float *cos_value) {
	const static float c[BLOCK_SIDE] = {(0.5f / sqrt(2)), 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
	float block_f[BLOCK_SIZE];
	for(int i = 0; i < BLOCK_SIZE; i++) {
		block_f[i] = block[i];
	}

	for(int y = 0; y < BLOCK_SIDE; y++) {
		float value[BLOCK_SIDE] = {0.0f};
		for(int x = 0; x < BLOCK_SIDE; x++) {
			for(int i = 0; i < BLOCK_SIDE; i++) {
				value[x] += c[i] * block_f[y * BLOCK_SIDE + i] * cos_value[x * BLOCK_SIDE + i];
			}
		}
		for(int i = 0; i < BLOCK_SIDE; i++) {
			block_f[y * BLOCK_SIDE + i] = value[i];
		}
	}

	for(int x = 0; x < BLOCK_SIDE; x++) {
		float value[BLOCK_SIDE] = {0.0f};
		for(int y = 0; y < BLOCK_SIDE; y++) {
			for(int i = 0; i < BLOCK_SIDE; i++) {
				value[y] += c[i] * block_f[x + i * BLOCK_SIDE] * cos_value[y * BLOCK_SIDE + i];
			}
		}
		for(int i = 0; i < BLOCK_SIDE; i++) {
			block_f[x + i * BLOCK_SIDE] = value[i];
		}
	}
	for(int i = 0; i < BLOCK_SIZE; ++i) {
		block[i] = roundf(block_f[i] + 128);
	}
}
/*
   void IDCT_block(int* block, float cos_table[][BLOCK_SIDE]) {
   float blockf[BLOCK_SIZE];
   for (int i = 0; i < BLOCK_SIZE; i++)
   blockf[i] = (float)block[i];
   for (int x = 0; x < BLOCK_SIDE; x++) {
   for (int y = 0; y < BLOCK_SIDE; y++) {
   float sum = 0;
   for (int u = 0; u < BLOCK_SIDE; u++) {
   for (int v = 0; v < BLOCK_SIDE; v++) {
   float c = ((u == 0) && (v == 0))? 0.5: 1;
   sum += c * blockf[x * BLOCK_SIDE + y] * cos_table[x][u] * cos_table[y][v];
   }
   }
   blockf[x * BLOCK_SIDE + y] = sum;
   }
   }
   for (int i = 0; i < BLOCK_SIZE; i++) {
   block[i] = roundf(blockf[i] + 128);
   printf("block[i]=%d\n", block[i]);
   }
   return;
   }*/

void IDCT(JpegImage* ptr) {
	// calculate cosine table
	float cos_table[BLOCK_SIZE];
	for (int x = 0; x < BLOCK_SIDE; x++) {
		for (int u = 0; u < BLOCK_SIDE; u++) {
			cos_table[x * BLOCK_SIDE + u] = cosf((2. * x + 1.) * (M_PI / 16.) * u); 
		}
	}
	int n_mcu = ptr->n_rows_mcu * ptr->n_cols_mcu;
	int n_tables[N_COMPONENTS];
	for (int c = 0; c < N_COMPONENTS; c++)
		n_tables[c] = ptr->subsampling[c][0] * ptr->subsampling[c][1];
	for (int c = 0; c < N_COMPONENTS; c++) {
		for (int mcu = 0; mcu < n_mcu; mcu++) {
			for (int t = 0 ;t < n_tables[c]; t++) {
				IDCT_block(ptr->blocks[c][mcu * n_tables[c] + t], cos_table); 
			}
		}
	}
	return;
}

BmpImage* convertColor(JpegImage* ptr) {
	BmpImage* bmp = new BmpImage;
	bmp->width = ptr->n_cols_mcu * ptr->Hmax * BLOCK_SIDE;
	bmp->height = ptr->n_rows_mcu * ptr->Vmax * BLOCK_SIDE;
	bmp->pad_width = bmp->width - ptr->width;
	bmp->pad_height = bmp->height - ptr->height;
	// allocate pixel memory
    bmp->pixel = new unsigned char [N_COMPONENTS * bmp->width * bmp->height];
	int sample_distance_h[N_COMPONENTS], sample_distance_v[N_COMPONENTS];

	for (int c = 0; c < N_COMPONENTS; c++) {
		sample_distance_h[c] = ptr->Hmax / ptr->subsampling[c][0];
		sample_distance_v[c] = ptr->Vmax / ptr->subsampling[c][1];
	}
	int n_tables[N_COMPONENTS] = {0};
	// number of tables in a block
	for (int c = 0; c < N_COMPONENTS; c++)
		n_tables[c] = ptr->subsampling[c][0] * ptr->subsampling[c][1];
	for (int i = 0; i < bmp->height; i++) {
		for (int j = 0; j < bmp->width; j++) {
			int YCrCb[N_COMPONENTS] = {};
			for (int c = 0; c < N_COMPONENTS; c++) {
				int block_h = j / (ptr->Hmax * BLOCK_SIDE);
				int block_v = i / (ptr->Vmax * BLOCK_SIDE);
				int block_index = block_v * ptr->n_cols_mcu + block_h;
				int table_h = (j - block_h * ptr->Hmax * BLOCK_SIDE) / (BLOCK_SIDE * sample_distance_h[c]);
				int table_v = (i - block_v * ptr->Vmax * BLOCK_SIDE) / (BLOCK_SIDE * sample_distance_v[c]);
				int table_index = table_v * ptr->subsampling[c][0] + table_h;
				int pixel_h = (j - block_h * ptr->Hmax * BLOCK_SIDE - table_h * BLOCK_SIDE) / sample_distance_h[c];
				int pixel_v = (i - block_v * ptr->Vmax * BLOCK_SIDE - table_v * BLOCK_SIDE) / sample_distance_v[c];
				int pixel_index = pixel_v * BLOCK_SIDE + pixel_h;
				YCrCb[c] = ptr->blocks[c][block_index * n_tables[c] + table_index][pixel_index];
			}
			int pixel_index = (bmp->height - i) * bmp->width + j;
			// order: B, G, R
			bmp->pixel[pixel_index*N_COMPONENTS] = clip(roundf(YCrCb[0] + 1.772f * (YCrCb[1] - 128)), 0, 255);
			bmp->pixel[pixel_index*N_COMPONENTS + 1] = clip(roundf(YCrCb[0] - 0.71414f * (YCrCb[2] - 128) - 0.34414f * (YCrCb[1] - 128)), 0, 255);
			bmp->pixel[pixel_index*N_COMPONENTS + 2] = clip(roundf(YCrCb[0] + 1.402f * (YCrCb[2] - 128)), 0, 255);
		}
	}
	return bmp;
}

JpegImage* ReadJpeg(FILE* fp) {
	JpegImage* ImagePtr = new JpegImage;
	unsigned char buffer[2];
	int end = 0;
	// SOI
	readBytes(fp, buffer, 2);
	if (buffer[0] != MarkerStart || buffer[1] != MarkerSOI)
		printf("SOI not found\n");
	// read headers
	while (!end) {
		readBytes(fp, buffer, 2);
		switch(buffer[1]) {
			case MarkerAPP0: 
				readAPP0(fp, ImagePtr);
				break;
			case MarkerDQT: 
				readDQT(fp, ImagePtr);
				break;
			case MarkerSOF: 
				readSOF(fp, ImagePtr);
				break;
			case MarkerDHT: 
				readDHT(fp, ImagePtr);
				break;
			case MarkerDRI: 
				readDRI(fp, ImagePtr);
				break;
			case MarkerSOS: 
				readSOS(fp, ImagePtr);
				end = readMCU(fp, ImagePtr);
				break;
			case MarkerEOI: 
				end = 1;
				break;
		}
	}
	return ImagePtr;
}

int main(int argc, char** argv) {
	// argv[1]: input image path, argv[2]: output image path
	FILE* fp = fopen(argv[1], "rb");
	JpegImage* jpeg_ptr;
	jpeg_ptr = ReadJpeg(fp);
	dequantize(jpeg_ptr);
	IDCT(jpeg_ptr);
	BmpImage* bmp = convertColor(jpeg_ptr);
    writeBMP(bmp, argv[2]);
	return 0;
}
