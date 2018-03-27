#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

#define SPATIAL_LIMIT 0.003
#define TIME_LIMIT 30000
#define MIN_BOUNDARY 0.01

// --------------------------------------------------------------------
// ---------------------   Progress Bar   -----------------------------

typedef struct progress {
	int max_value;
	int size;
	int current;
	clock_t changedIn;
	clock_t startTime;
} ProgressBar;

ProgressBar* newProgressBar(int max_value, int size) {
	ProgressBar* bar = (ProgressBar*) malloc(sizeof(ProgressBar));
	bar->max_value = max_value;
	bar->size = size;
	bar->current = 0;
	bar->changedIn = clock();
	bar->startTime = -1;
	return bar;
}

float getPercentage(ProgressBar* bar) {
    return bar->current / (float) bar->max_value;
}

void flushProgress(ProgressBar* bar) {
    bar->changedIn = clock();
    int totalTime = (int) ( (float)(bar->changedIn - bar->startTime) / CLOCKS_PER_SEC );
    int hours = totalTime / 3600;
    int minutes = (totalTime % 3600) / 60;
    int seconds = totalTime % 60;
    float percentage = getPercentage(bar);
    
    printf("\r%02d:%02d:%02d - [", hours, minutes, seconds);
    int i;
    for(i = 0; i < round(percentage * bar->size); i++)
        printf("#");
    for(i = 0; i < round((1 - percentage) * bar->size); i++)
        printf(" ");
    printf("] ( %.2f %% )", percentage*100);
}

void draw(ProgressBar* bar) {
    clock_t now = clock();
    if (bar->startTime == -1)
        bar->startTime = now;
    float deltaTime = (float)(now - bar->changedIn) / CLOCKS_PER_SEC;
    if (deltaTime >= 0.5)
        flushProgress(bar);
}

void set(ProgressBar* bar, int n) {
	bar->current = n;
	draw(bar);
}

void step(ProgressBar* bar) {
    bar->current ++;
    draw(bar);
}

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

double distance(Point* p1, Point* p2) {
    return sqrt(pow(p2->lat - p1->lat, 2) + pow(p2->lng - p1->lng, 2));
}

void printPoint(Point* p) {
    printf("Id: %d\tLat: %lf\tLng: %lf\tTimestamp: %lld\n", p->taxiId, p->lat, p->lng, p->t);
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

void printTrajectory(Trajectory* t) {
    int i;
    for (i = 0; i < t->filled; i++)
        printPoint(t->points[i]);
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

int getTotalNumberOfPoints(char* inputFileName) {
    FILE* input = fopen(inputFileName, "r");
    if (input == NULL) return -1;
    
    int counter = 0;
    char line[128];
    while( fgets(line, 128, input) ) counter++;
    fclose(input);
    return counter - 1;
}

char* getOutputFileName(char* inputFileName) {
    char* outputFileName = (char*) malloc(sizeof(char)*(strlen(inputFileName) + 8));
    strcpy(outputFileName, "cfixed_");
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

typedef struct {
    FILE* output;
    int nextId;
} TrajectoryWriter;

TrajectoryWriter* newTrajectoryWriter(FILE* output) {
    TrajectoryWriter* writer = (TrajectoryWriter*) malloc(sizeof(TrajectoryWriter));
    writer->output = output;
    writer->nextId = 0;
    fprintf(output, "driver_id;id;lat;lng;timestamp\n");
    return writer;
}

void writeTrajectory(TrajectoryWriter* writer, Trajectory* t) {
    int i;
    for (i = 0; i < t->filled; i++) {
        Point* p = t->points[i];
        fprintf(writer->output, "%d;%d;%.8lf;%.8lf;%lld\n", p->taxiId, writer->nextId, p->lat, p->lng, p->t);
    }
    writer->nextId ++;
}

// -----------------------------------------------------------------------------------
// -------------------------   Actual Processing   -----------------------------------

void swapPoints(Point** points, int i, int j) {
    if (i == j) return;
    Point* aux = points[i];
    points[i] = points[j];
    points[j] = aux;
}

void quickSort(Point** points, int start, int end) {
    if (end - start > 1) {
        long long pivot = points[end-1]->t;
        int i = start, j = start;
        while(j < end) {
            if (points[j]->t <= pivot) {
                swapPoints(points, i, j);
                i++;
            }
            j++;
        }
        quickSort(points, start, i-1);
        quickSort(points, i, end);
    }
}

void sortTrajectory(Trajectory* t) {
    quickSort(t->points, 0, t->filled);
}

int isValid(Trajectory* trajectory) {
    if (trajectory == NULL || trajectory->filled == 0)
        return 0;
    Point* p = trajectory->points[0];
    double  max_lat = p->lat, 
            min_lat = p->lat, 
            max_lng = p->lng, 
            min_lng = p->lng;
    int i;
    for (i = 1; i < trajectory->filled; i++) {
        p = trajectory->points[i];
        if (p->lat > max_lat) max_lat = p->lat;
        if (p->lat < min_lat) min_lat = p->lat;
        if (p->lng > max_lng) max_lng = p->lng;
        if (p->lng < min_lng) min_lng = p->lng;
        if (max_lat - min_lat > MIN_BOUNDARY || max_lng - min_lng > MIN_BOUNDARY)
            return 1;
    }

    return 0;
}

int getClosestPointIndex(Trajectory* t, Point* p, int rangeStart, int rangeEnd) {
    if (rangeStart >= t->filled)
        return t->filled;
    int i;
    int minIndex = rangeStart;
    double minDistance = distance(p, t->points[minIndex]);
    for (i = rangeStart+1; i < rangeEnd; i++) {
        double newDistance = distance(p, t->points[i]);
        if (newDistance < minDistance) {
            minDistance = newDistance;
            minIndex = i;
        }
    }
    return minIndex;
}

void sliceNspliceNsave(Trajectory* originalTrajectory, TrajectoryWriter* writer) {
    int start = 0, end = 1;
    sortTrajectory(originalTrajectory);
    Trajectory* t = newTrajectory();
    Point* p;
    while (start < originalTrajectory->filled) {
        p = originalTrajectory->points[start];
        addPoint(t, p);
        while(end < originalTrajectory->filled && originalTrajectory->points[end]->t < p->t + TIME_LIMIT) 
            end++;
        int minIndex = getClosestPointIndex(originalTrajectory, p, start+1, end);
        if (minIndex == originalTrajectory->filled || distance(p, originalTrajectory->points[minIndex]) > SPATIAL_LIMIT) {
            if (isValid(t))
                writeTrajectory(writer, t);
            t = newTrajectory();
        }
        start = minIndex;
    }
}

void readAndProcess(char* inputFileName) {
    int totalNumberOfPoints = getTotalNumberOfPoints(inputFileName);
    char* outputFileName = getOutputFileName(inputFileName);
    FILE* input = fopen(inputFileName, "r");
    FILE* output = fopen(outputFileName, "w");
    if (input == NULL || output == NULL) {
        printf("Error opening files\n");
        return;
    }

    printf("Fixing: %s => %s\n", inputFileName, outputFileName);

    ProgressBar* progress = newProgressBar(totalNumberOfPoints, 50);
    draw(progress);
    TrajectoryReader* reader = newTrajectoryReader(input);
    TrajectoryWriter* writer = newTrajectoryWriter(output);

    Trajectory* t;

    while((t = readTrajectory(reader)) != NULL) {
        sliceNspliceNsave(t, writer);
        set(progress, progress->current + t->filled);
        freeTrajectory(t);
    }

    flushProgress(progress);

    fclose(input);
    fclose(output);
}


// -----------------------------------------------------------------------------
// -------------------------------   Main   ------------------------------------

int main(int argc, char** argv) {

    if (argc != 2) {
        printf("Invalid number of arguments, expected 2, found %d\n", argc);
        return 1;
    }
    readAndProcess(argv[1]);
	return 0;
}
