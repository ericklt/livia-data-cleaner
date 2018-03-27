#include <stdio.h>
#include <stdlib.h>
#include <string.h>	

// ----------------------------------------------------------------------
// ----------------------------   Point   -------------------------------

typedef struct {
    int taxiId;
    double lat;
    double lng;
    long long t;
} Point;

Point* newPoint(int taxiId, double lat, double lng, long long t) {
    Point* p = (Point*) malloc(sizeof(Point));
    p->taxiId = taxiId;
    p->lat = lat;
    p->lng = lng;
    p->t = t;
    return p;
}

// ---------------------------------------------------------------------------
// -----------------------------   Trajectory   ------------------------------

typedef struct {
    int id;
    int size;
    int filled;
    Point** points;
} Trajectory;

Trajectory* newTrajectory() {
    Trajectory* t = (Trajectory*) malloc(sizeof(Trajectory));
    t->id = -1;
    t->size = 100;
    t->filled = 0;
    t->points = (Point**) malloc(sizeof(Point*) * 100);
    return t;
}

void addPoint(Trajectory* t, Point* p) {
    if (t->filled >= t->size) {
        t->size += 100;
        t->points = (Point**) realloc(t->points, sizeof(Point*) * t->size);
    }
    t->points[t->filled] = p;
    t->filled ++;
}

Point* getPoint(Trajectory* t, int index) {
    if (index >= t->filled) return NULL;
    return t->points[index];
}

void freeTrajectory(Trajectory* t) {
    int i;
    for (i = 0; i < t->filled; i++)
        free(t->points[i]);
    free(t->points);
    free(t);
}


// ---------------------------------------------------------------------------
// ---------------------------   Utils   -------------------------------------

char* getOutputFileName(char* inputFileName) {
    char* outputFileName = (char*) malloc(sizeof(char)*(strlen(inputFileName) + 11));
    strcpy(outputFileName, "converted_");
    strcat(outputFileName, inputFileName);
    return outputFileName;
}

Point* readPoint(FILE* input) {
    int id;
    double lat, lng;
    long long timestamp;
    char* line = (char*) malloc(sizeof(char) * 128);
    if (fgets(line, 128, input) == NULL)
        return NULL;
    strtol(line, &line, 10);   line++;
    id = strtol(line, &line, 10);   line++;
    lat = strtod(line, &line);      line++;
    lng = strtod(line, &line);      line++;
    timestamp = strtoll(line, &line, 10);
    Point* p = newPoint(id, lat, lng, timestamp);
    return p;
}

typedef struct {
    FILE* input;
    Point* buffer;
} TrajectoryReader;

TrajectoryReader* newTrajectoryReader(FILE* input) {
    TrajectoryReader* reader = (TrajectoryReader*) malloc(sizeof(TrajectoryReader));
    reader->input = input;
    reader->buffer = NULL;
    readPoint(input);
    return reader;
}

Trajectory* readTrajectory(TrajectoryReader* reader) {
    int tId;
    Point* p;
    Trajectory* trajectory = newTrajectory();
    if (reader->buffer == NULL) {
        p = readPoint(reader->input);
        if (p == NULL)
            return NULL;
    } else
        p = reader->buffer;

    tId = p->taxiId;
    do {
        addPoint(trajectory, p);
        p = readPoint(reader->input);
    } while (p != NULL && p->taxiId == tId);
    reader->buffer = p;
    return trajectory;
}

void writeTrajectory(FILE* output, Trajectory* t) {
	fprintf(output, "%d", t->id);
    int i;
    for (i = 0; i < t->filled; i++) {
        Point* p = t->points[i];
        fprintf(output, ";%.8lf;%.8lf", p->lat, p->lng);
    }
    fprintf(output, "\n");
}

// -----------------------------------------------------------------------------
// -------------------------------   Main   ------------------------------------

void convert(char* inputFileName) {
    char* outputFileName = getOutputFileName(inputFileName);
    FILE* input = fopen(inputFileName, "r");
    FILE* output = fopen(outputFileName, "w");
    if (input == NULL || output == NULL) {
        printf("Error opening files\n");
        return;
    }

    printf("Converting: %s => %s\n", inputFileName, outputFileName);

    TrajectoryReader* reader = newTrajectoryReader(input);

    Trajectory* t;

    while((t = readTrajectory(reader)) != NULL) {
        writeTrajectory(output, t);
        freeTrajectory(t);
    }

    fclose(input);
    fclose(output);
}

int main(int argc, char** argv) {

    if (argc != 2) {
        printf("Invalid number of arguments, expected 2, found %d\n", argc);
        return 1;
    }
    convert(argv[1]);
	return 0;
}
