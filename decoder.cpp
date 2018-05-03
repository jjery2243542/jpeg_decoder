#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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
    int block[BLOCK_SIZE];
} JpegImage;

typedef struct BitReader {
    int buffer;
    int remain;
    BitReader() {
        buffer = 0; remain = 0;
    };
    int read_bit(FILE *fp) {
        int ret;
        unsigned char c;
        if (remain == 0) {
            fread(&c, 1, 1, fp);
            buffer = 0;
            buffer |= (int)c;
            remain += 8;
        }
        ret = ((buffer >> (remain - 1)) & 1);
        remain--;
        return ret;
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
        //printf("q_table_id=%d\n", id);
        ptr->n_q_tables = id + 1;
        Fill_Q_table(ptr->q_table[id], buffer + 1);
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
    //printf("%d\n", length);
    readBytes(fp, buffer, 1);
    precision = (int)buffer[0];
    //printf("%d\n", precision);
    readBytes(fp, buffer, 2);
    ptr->height = ConcatBytes(buffer[0], buffer[1]);
    readBytes(fp, buffer, 2);
    ptr->width = ConcatBytes(buffer[0], buffer[1]);
    // printf("%d %d\n", ptr->height, ptr->width);
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
        //printf("component=%d, subsampling=(%d, %d)\n", i, ptr->subsampling[i][0], ptr->subsampling[i][1]);
        // calculate HMax and VMax
        if (ptr->subsampling[i][0] > ptr->Hmax)
            ptr->Hmax = ptr->subsampling[i][0];
        if (ptr->subsampling[i][1] > ptr->Vmax)
            ptr->Vmax = ptr->subsampling[i][1];
        ptr->q_table_id[i] = (int)buffer[2];
        //printf("%d %d %d\n", ptr->subsampling[i][0], ptr->subsampling[i][1], ptr->q_table_id[i]);
    }
    printf("Hmax=%d, Vmax=%d\n", ptr->Hmax, ptr->Vmax);

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
    //printf("%d\n", length);
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
        printf("type=%d, table_id=%d\n", type, table_id);
        auto& map = ptr->huffman_tables[type][table_id];
        for (int i = 0; i < MAX_CODE_LENGTH; i++) {
            num_leaves[i] = (int)buffer[i + 1];
            printf("length=%d, num=%d\n", i + 1, num_leaves[i]);
            if (num_leaves[i] > 0)
                max_length = i + 1;
        }
        code = 0;
        for (int len = 1; len <= max_length; len++) {
            printf("len=%d, num_leaves=%d\n", len, num_leaves[len - 1]);
            for (int i = 0; i < num_leaves[len - 1]; i++) {
                readBytes(fp, buffer, 1);
                bytes_left--;
                map[ConcatInt(len, code)] = (int)buffer[0];
                printf("len=%d, code=%x, weights=%d\n", len, code, (int)buffer[0]);
                code++;
            }
            code = code << 1;
        }
    }
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
    //printf("length=%d\n", length);
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
    //printf("%x %x %x\n", buffer[0], buffer[1], buffer[2]);
    return;
}

int translate(int weight, BitReader& BR, FILE* fp) {
    int value = 0, translation = 0;
    int bit = 0, is_negative = 0;
    for (int len = 0; len < weight; len++) {
        bit = BR.read_bit(fp);
        //printf("bit=%d ", bit);
        value = (value << 1) | bit;
        if (len == 0)
            is_negative = ((bit == 0)? 1 : 0); 
    }
    //printf("is_nagtive=%d, value=%d\n", is_negative, value);
    if (is_negative) 
        translation = -(value^((1 << weight) - 1));
    else
        translation = value;
    //printf("translation=%d\n", translation);
    return translation;
}

void decodeMCU(FILE* fp, auto& dc_table, auto& ac_table, JpegImage* ptr, BitReader& BR, int* table) {
    unsigned char buffer[2];
    int count = 0, terminate = 0;
    while (count >= BLOCK_SIZE || terminate) {
        int length = 0, code = 0, find = 0, weight = 0, translation = 0;
        int n_zeros = 0, ac_value = 0;
        auto& huffman_table = (count == 0)? dc_table: ac_table;
        int bit = 0;
        printf("block index=%d\n", count);
        while (!find) {
            bit = BR.read_bit(fp);
            printf("bit=%d ", bit);
            code = (code << 1) | bit;
            length++;
            printf("length=%d, code=%x\n", length, code);
            auto got = huffman_table.find(ConcatInt(length, code));
            // code found
            if (got != huffman_table.end()) {
                printf("gotten, code=%x\n", code);
                weight = got->second;
                find = 1; 
            }
        }
        printf("weight=%d\n", weight);
        // AC terminate code
        if (weight == 0) {
            terminate = 1;
            continue;
        }
        // DC 
        if (count == 0) {
            // read weight bits of data
            translation = translate(weight, BR, fp);
            table[count] = translation;
            count++;
        } else {
            // high 4 bits are the running zero, low 4 bits are the value
            n_zeros = weight >> 4;
            weight = weight & 0xf;
            //printf("AC: n_zeros=%d, weight=%d\n", n_zeros, weight);
            translation = translate(weight, BR, fp);
            // fill in AC value
            for (int zero = 0; zero < n_zeros; zero++, count++) 
                table[count] = 0; 
            table[count] = translation;
            count++;
        }
    }
    return;
}

void readMCU(FILE* fp, JpegImage* ptr) {
    unsigned char buffer[BLOCK_SIZE];
    vector<int*> tables[N_COMPONENTS];
    BitReader BR;
    int n_rows_mcu = BLOCK_SIDE * ptr->Vmax;
    int n_cols_mcu = BLOCK_SIDE * ptr->Hmax;
    printf("height=%d, width=%d\n", ptr->height, ptr->width);
    int n_mcu = ceil(ptr->height / n_rows_mcu) * ceil(ptr->width / n_cols_mcu); 
    printf("n_mcu=%d\n", n_mcu);
    for (int mcu = 0; mcu < n_mcu; mcu++) {
        //printf("mcu_index=%d\n", mcu);
        for (int c = 0; c < N_COMPONENTS; c++) {
            int dc_table_id = ptr->h_table_id[DC][c];
            int ac_table_id = ptr->h_table_id[AC][c];
            //printf("dc_table_id=%d, ac_table_id=%d\n", dc_table_id, ac_table_id);
            auto& dc_table = ptr->huffman_tables[DC][dc_table_id];
            auto& ac_table = ptr->huffman_tables[AC][ac_table_id];
            int n_tables = ptr->subsampling[c][0] * ptr->subsampling[c][1];
            int* table = new int(BLOCK_SIZE);
            int previous_dc = 0;
            for (int t = 0; t < n_tables; t++) {
                 decodeMCU(fp, dc_table, ac_table, ptr, BR, table);
                 table[0] += previous_dc;
                 previous_dc = table[0];
                 tables[c].push_back(table);
            } 
        }
    }
    return;
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
            case MarkerAPP0: {
                printf("first\n");
                readAPP0(fp, ImagePtr);
                break;
            }
            case MarkerDQT: {
                printf("second\n");
                readDQT(fp, ImagePtr);
                break;
            }
            case MarkerSOF: {
                printf("third\n");
                readSOF(fp, ImagePtr);
                break;
            }
            case MarkerDHT: {
                printf("fourth\n");
                readDHT(fp, ImagePtr);
                break;
            }
            case MarkerDRI: {
                printf("fifth\n");
                readDRI(fp, ImagePtr);
                break;
            }
            case MarkerSOS: {
                printf("sixth\n");
                readSOS(fp, ImagePtr);
                readMCU(fp, ImagePtr);
                end = 1;
                break;
            }
            case MarkerEOI: {
                printf("end\n");
                end = 1;
                break;
            }
        }
    }
    readBytes(fp, buffer, 1);
    printf("%x", buffer[0]);
    return ImagePtr;
}

int main(int argc, char** argv) {
    // argv[1]: input image path, argv[2]: output image path
    FILE* fp = fopen(argv[1], "rb");
    JpegImage* jpeg_ptr;
    jpeg_ptr = ReadJpeg(fp);
    return 0;
}
