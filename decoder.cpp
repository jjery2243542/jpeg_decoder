#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <unordered_map>
#include <vector>

#define BLOCK_SIDE 8
#define BLOCK_SIZE 64
#define MAX_Q_TABLE 64
#define N_COMPONENTS 3
#define MAX_CODE_LENGTH 16
#define N_TYPES 2
#define N_DECODE_TREES 2 

using namespace std;

const unsigned char MarkerStart = 0xff;
const unsigned char MarkerSOI = 0xd8;
const unsigned char MarkerAPP0 = 0xe0;
const unsigned char MarkerDQT = 0xdb;
const unsigned char MarkerSOF = 0xc0;
const unsigned char MarkerDHT = 0xc4;

typedef struct JpegImage {
    int unit;
    int density[2];
    int thumbnail[2];
    int n_q_tables;
    int q_table[MAX_Q_TABLE][BLOCK_SIZE];
    int height, width;
    int subsampling[N_COMPONENTS][2];
    int q_table_id[N_COMPONENTS]; 
    unordered_map <int, int> huffman_tables[N_TYPES][N_DECODE_TREES]; 
} JpegImage;

void readBytes(FILE* fp, auto* buffer, size_t n) {
    fread(buffer, 1, n, fp);
    return;
}

int ConcatBytes(auto high, auto low) {
    return (int)((high << 8) | low);
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
    //printf("%d %d\n", ptr->height, ptr->width);
    // color division must be 3
    readBytes(fp, buffer, 1);
    n_components = (int)buffer[0];
    if (n_components != N_COMPONENTS)
        printf("n_components not matching\n");
    for (int i = 0; i < n_components; i++) {
        readBytes(fp, buffer, 3);
        if ((int)buffer[0] != i + 1) 
            printf("frame id not matching\n");
        ptr->subsampling[i][0] = (int)(buffer[1] >> 4);
        ptr->subsampling[i][1] = (int)(buffer[1] & 0xf);
        ptr->q_table_id[i] = (int)buffer[2];
        //printf("%d %d %d\n", ptr->subsampling[i][0], ptr->subsampling[i][1], ptr->q_table_id[i]);
    }
    return;
}

void readDHT(FILE* fp, JpegImage* ptr) {
    unsigned char buffer[MAX_CODE_LENGTH + 1];
    int length, bytes_left;
    int type, table_id;
    int num_leaves[MAX_CODE_LENGTH] = {0}, max_length = 0;
    readBytes(fp, buffer, 2);
    length = ConcatBytes(buffer[0], buffer[1]);
    printf("%d\n", length);
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
            printf("length=%d, num=%d\n", i + 1, num_leaves[i]);
            if (num_leaves[i] > 0)
                max_length = i + 1;
        }
        for (int len = 1, code = 0; len <= max_length; len++) {
            for (int i = 0; i < num_leaves[len - 1]; i++) {
                readBytes(fp, buffer, 1);
                printf("code=%x, weights=%d\n", code, (int)buffer[0]);
                map[code] = (int)buffer[0];
                code++;
            }
            code = code << 1;
        }
        return;
    }
    return;
}

JpegImage* ReadJpeg(FILE* fp) {
    JpegImage* ImagePtr = (JpegImage*)malloc(sizeof(JpegImage));
    unsigned char buffer[2];
    int mcu_start = 0;
    // SOI
    readBytes(fp, buffer, 2);
    if (buffer[0] != MarkerStart || buffer[1] != MarkerSOI)
        printf("SOI not found\n");
    while (!mcu_start) {
        readBytes(fp, buffer, 2);
        switch(buffer[1]) {
            case MarkerAPP0:
                printf("first\n");
                readAPP0(fp, ImagePtr);
                break;
            case MarkerDQT:
                printf("second\n");
                readDQT(fp, ImagePtr);
                break;
            case MarkerSOF:
                printf("third\n");
                readSOF(fp, ImagePtr);
                break;
            case MarkerDHT:
                printf("fourth\n");
                readDHT(fp, ImagePtr);
                mcu_start = 1;
        }
    }
    return ImagePtr;
}

int main(int argc, char** argv) {
    // argv[1]: input image path, argv[2]: output image path
    FILE* fp = fopen(argv[1], "rb");
    JpegImage* jpeg_ptr;
    jpeg_ptr = ReadJpeg(fp);
    return 0;
}
