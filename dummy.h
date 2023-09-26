#ifndef DUMMY_H
#define DUMMY_H
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define logging 1
#define map(x, in_min, in_max, out_min, out_max) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
#define MAX_LINE_LEN 1024
#define MAX_FIELD_LENGTH 256
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
#define A10 10
#define OUTPUT 0
class SerialIO {
public:
    void print(char* text) {
        printf("%s", text);
        strcat(log_str, text);
    }
    void print(const float& value) {
        printf("%f", value);
        char field[MAX_FIELD_LENGTH];
        sprintf(field, "%f", value);
        strcat(log_str, field);
    }
    void println(char* text) {
       printf("%s\n", text);
        strcat(log_str, strcat(text, "\n"));
    }
    void println(const float& value) {
        printf("%f\n", value);
        char field[MAX_FIELD_LENGTH];
        sprintf(field, "%f\n", value);
        strcat(log_str, field);
    }
    void println() {
        printf("\n");
        strcat(log_str, "\n");
    }
    void begin(int baudrate) {
        printf("Serial.begin(%d)\n", baudrate);
    }
};
void setup();
void loop();
SerialIO Serial;
int ir_raw[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char log_str[MAX_LINE_LEN];
int TCCR3B, TCCR3A, ICR3, OCR3A, OCR3B, TCNT3;
int analogRead(int pin) {
    return ir_raw[pin];
}
int pinMode(int pin, int mode) {
    return 0;
}
int analogWrite(int pin, int value) {
    return 0;
}
int main(int arc, char**argv) {
    char fname[100], line[MAX_LINE_LEN], field[MAX_FIELD_LENGTH];
    int line_count = 0;
    if (arc < 3) {
        printf("Usage: %s <input filename> <output filename>\n", argv[0]);
        return 1;
    }
    strcpy(fname, argv[1]);
    if(strchr(fname, '.') == NULL) {
        strcat(fname, ".csv");
    }
    FILE *fp = fopen(fname, "r");
    FILE *fp2 = fopen(argv[2], "w");
    if (fp == NULL) {
        printf("Error opening file %s\n", fname);
        return 1;
    }
    // Read and discard the header line
    if (fgets(line, sizeof(line), fp) == NULL) {
        perror("Error reading header line");
        fclose(fp);
        return 1;
    }

    setup();
    while (fgets(line, sizeof(line), fp) != NULL) {
        char *token;
        int ir_num = 0;
        log_str[0] = '\0';

        // Tokenize the line based on commas
        token = strtok(line, ",");
        while (token != NULL && ir_num < 11) {
            ir_raw[ir_num] = atoi(token);
            token = strtok(NULL, ",");
            ir_num++;
        }
        if (logging) {
            for (int i = 0; i < 11; ++i) {
                sprintf(field, "%d, ", ir_raw[i]);
                strcat(log_str, field);
            }
            log_str[strlen(log_str)-2] = '\n';
            log_str[strlen(log_str)-1] = '\0';
        }
        loop();
        if (logging) {
            fprintf(fp2, "%s\n", log_str);
        }
    }
    return 0;
}
#endif