#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <pthread.h>

#define R 6378137 // Radius of earth in m

#define MAX_SPEED 100  // Km/h
#define MAX_ANGULAR_SPEED (MAX_SPEED / (R * 3.6)) * 180 / M_PI
#define TIME_LIMIT 30 // seconds
#define MIN_FULL_TRAJ_BOUNDARY 0.05
#define MIN_BOUNDARY 0.005 // degrees

#define max(a,b) \
    ({  __typeof__ (a) _a = (a); \
        __typeof__ (b) _b = (b); \
        _a > _b ? _a : _b; })

#define min(a,b) \
    ({  __typeof__ (a) _a = (a); \
        __typeof__ (b) _b = (b); \
        _a < _b ? _a : _b; })

#define toKmph(mps) 3.6 * mps

// --------------------------------------------------------------------
// -----------------------   StopWatch   ------------------------------

typedef struct {
    char* name;
    double current;
    double start;
    int running;
} StopWatch;

StopWatch* newStopWatch(char* name) {
    StopWatch* watch = (StopWatch*) malloc(sizeof(StopWatch));
    watch->name = name;
    watch->current = 0;
    watch->running = 0;
}

void startClock(StopWatch* watch) {
    watch->start = clock();
    watch->running = 1;
}

void stopClock(StopWatch* watch) {
    if (watch->running) {
        watch->current += (clock() - watch->start) / CLOCKS_PER_SEC;
        watch->running = 0;
    }
}

double getTime(StopWatch* watch) {
    int time = watch->current;
    if (watch->running)
        time += (clock() - watch->start) / CLOCKS_PER_SEC;
    return time;
}

// --------------------------------------------------------------------
// ---------------------   Progress Bar   -----------------------------

typedef struct progress {
	long long max_value;
	int size;
	long long current;
	double changedIn;
	StopWatch* watch;
} ProgressBar;

ProgressBar* newProgressBar(long long max_value, int size, StopWatch* watch) {
	ProgressBar* bar = (ProgressBar*) malloc(sizeof(ProgressBar));
	bar->max_value = max_value;
	bar->size = size;
	bar->current = 0;
	bar->changedIn = clock();
	bar->watch = watch;
	return bar;
}

float getPercentage(ProgressBar* bar) {
    return bar->current / (float) bar->max_value;
}

void flushProgress(ProgressBar* bar) {
    bar->changedIn = clock();
    int totalTime = round(getTime(bar->watch));
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
    fflush(stdout);
}

void draw(ProgressBar* bar) {
    double now = clock();
    float deltaTime = (now - bar->changedIn) / CLOCKS_PER_SEC;
    if (deltaTime >= 1)
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
// ----------------------   Lat / Long Utils   --------------------------

double deg2rad(double deg) {
  return (deg * M_PI / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

double distanceInMeters(double lat1, double lng1, double lat2, double lng2) {
    double  rLat1 = deg2rad(lat1), 
            rLat2 = deg2rad(lat2), 
            rLng1 = deg2rad(lng1), 
            rLng2 = deg2rad(lng2);
    double dLat = rLat2 - rLat1;
    double dLng = rLng2 - rLng1;

    double a = sin(dLat/2) * sin(dLat/2) + cos(rLat1) * cos(rLat2) * sin(dLng/2) * sin(dLng/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return R * c * 1000; // meters
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
    // return sqrt(pow(p2->lat - p1->lat, 2) + pow(p2->lng - p1->lng, 2));
    return hypot(p2->lat - p1->lat, p2->lng - p1->lng);
    // return distanceInMeters(p1->lat, p1->lng, p2->lat, p2->lng);
}

double time_difference(Point* p1, Point* p2) {
    return abs(p2->t - p1->t) / (double) 1000;
}

double angular_speed(Point* p1, Point* p2) {
    return distance(p1, p2) / time_difference(p1, p2);
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
    double minLat, minLng, maxLat, maxLng;
} Trajectory;

Trajectory* newTrajectory();
void addPoint(Trajectory*, Point*);
Point* getPoint(Trajectory*, int);
void printTrajectory(Trajectory*);
void softFreeTrajectory(Trajectory*);

Trajectory* newTrajectory() {
    Trajectory* t = (Trajectory*) malloc(sizeof(Trajectory));
    t->id = -1;
    t->size = 100;
    t->filled = 0;
    t->points = (Point**) malloc(sizeof(Point*) * 100);
    return t;
}

void addPoint(Trajectory* t, Point* p) {
    // if (t->filled > 0 && getPoint(t, t->filled-1)->t == p->t) return;
    if (t->filled >= t->size) {
        t->size += 100;
        t->points = (Point**) realloc(t->points, sizeof(Point*) * t->size);
    }
    t->points[t->filled] = p;
    t->filled ++;
    if (t->filled == 1) {
        t->minLat = t->maxLat = p->lat;
        t->minLng = t->maxLng = p->lng;
    } else {
        t->minLat = min(t->minLat, p->lat);
        t->minLng = min(t->minLng, p->lng);
        t->maxLat = max(t->maxLat, p->lat);
        t->maxLng = max(t->maxLng, p->lng);
    }
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

void softFreeTrajectory(Trajectory* t) {
    free(t->points);
    free(t);
}

void freeTrajectory(Trajectory* t) {
    int i;
    for (i = 0; i < t->filled; i++)
        free(t->points[i]);
    softFreeTrajectory(t);
}

int isValid(Trajectory* trajectory) {
    if (trajectory == NULL || trajectory->filled == 0)
        return 0;

    return trajectory->maxLat - trajectory->minLat > MIN_BOUNDARY || trajectory->maxLng - trajectory->minLng > MIN_BOUNDARY;
}

// ---------------------------------------------------------------------------
// ---------------------------   Utils   -------------------------------------

char* skipLine(FILE* input) {
    char line[128];
    return fgets(line, 128, input);
}

int getTotalNumberOfPoints(char* inputFileName) {
    FILE* input = fopen(inputFileName, "r");
    if (input == NULL) return -1;
    
    int counter = 0;
    while( skipLine(input) ) counter++;
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
    sscanf(line, "%d;%lf;%lf;%lld", &id, &lat, &lng, &timestamp);
    // if (feof(input)) return NULL;
    // fscanf(input, "%d;%lf;%lf;%lld\n", &id, &lat, &lng, &timestamp);
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
    skipLine(input);
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
        if (i > 0 && p->t != t->points[i-1]->t)
            fprintf(writer->output, "%d;%d;%.8lf;%.8lf;%lld\n", p->taxiId, writer->nextId, p->lat, p->lng, p->t);
    }
    writer->nextId ++;
}

// -----------------------------------------------------------------------------------
// -------------------------   Actual Processing   -----------------------------------

int comparePoints(const void *e1, const void *e2) {
    Point* p1 = *(Point**) e1;
    Point* p2 = *(Point**) e2;
    return p1->t - p2->t;
}

void sortTrajectory(Trajectory* t) {
    qsort(t->points, t->filled, sizeof(Point*), comparePoints);
}

int getClosestPointIndex(Trajectory* t, Point* p, int rangeStart, int rangeEnd) {
    while(rangeStart < rangeEnd && t->points[rangeStart]->t == p->t) 
        rangeStart++;
    if (rangeStart >= t->filled)
        return t->filled;
    int i;
    int minIndex = rangeStart;
    double minDistance = distance(p, t->points[minIndex]);
    for (i = rangeStart+1; i < rangeEnd; i++) {
        if (t->points[i]->t == p->t) continue;
        double newDistance = distance(p, t->points[i]);
        if (newDistance < minDistance) {
            minDistance = newDistance;
            minIndex = i;
        }
    }
    return minIndex;
}

typedef struct {
    Trajectory* t;
    TrajectoryWriter* writer;
    StopWatch** watches;
} AuxParameter;

AuxParameter* newAuxParameter(Trajectory* t, TrajectoryWriter* writer, StopWatch** watches) {
    AuxParameter* param = (AuxParameter*) malloc(sizeof(AuxParameter));
    param->t = t;
    param->writer = writer;
    param->watches = watches;
    return param;
}

void sliceNspliceNsave(Trajectory* originalTrajectory, TrajectoryWriter* writer, StopWatch** watches) {
    int start = 0, end = 1;
    if (!isValid(originalTrajectory)) return;
    // printf("\tSorting... ");
    startClock(watches[1]);
    sortTrajectory(originalTrajectory);
    stopClock(watches[1]);
    // printf("Done\n");
    Trajectory* t = newTrajectory();
    Point* p;
    startClock(watches[2]);
    while (start < originalTrajectory->filled) {
        p = originalTrajectory->points[start];
        addPoint(t, p);
        // if (writer->nextId == 320)
        //     printPoint(p);
        while(end < originalTrajectory->filled && time_difference(originalTrajectory->points[end], p) < TIME_LIMIT) 
            end++;
        startClock(watches[4]);
        int minIndex = getClosestPointIndex(originalTrajectory, p, start+1, end);
        // if (minIndex >= end) printf("\n\nIMPOSSIBRU!!!!!!\n\n");
        Point* closest = NULL;
        if (minIndex < end)
            closest = originalTrajectory->points[minIndex];
        stopClock(watches[4]);
        // if (closest != NULL &&  distance(p, closest) > 0.001 && angular_speed(p, closest) > MAX_ANGULAR_SPEED <= MAX_ANGULAR_SPEED) {
        //     printf(" -- %lf / %lf -> ", distance(p, closest), time_difference(p, closest));
        //     printf("speed: %.8lf\n", angular_speed(p, closest));
        // }
        if (closest == NULL || angular_speed(p, closest) > MAX_ANGULAR_SPEED) {
            if (isValid(t)) {
                startClock(watches[3]);
                writeTrajectory(writer, t);
                stopClock(watches[3]);
            }
            
            softFreeTrajectory(t);
            t = newTrajectory();
        }
        start = minIndex;
    }
    stopClock(watches[2]);
}

void* sliceNspliceNsaveOnThread(void* param) {
    AuxParameter* auxPar = (AuxParameter*) param;
    sliceNspliceNsave(auxPar->t, auxPar->writer, auxPar->watches);
    return (void*) NULL;
}

void* readTrajectoryOnThread(void* reader) {
    return (void*) readTrajectory((TrajectoryReader*) reader);
}

void readAndProcess(char* inputFileName, StopWatch** watches) {
    startClock(watches[5]);
    long long totalNumberOfPoints = getTotalNumberOfPoints(inputFileName);
    stopClock(watches[5]);
    // printf("N points: %lld\n", totalNumberOfPoints);
    char* outputFileName = getOutputFileName(inputFileName);
    FILE* input = fopen(inputFileName, "r");
    FILE* output = fopen(outputFileName, "w");
    if (input == NULL || output == NULL) {
        printf("Error opening files\n");
        return;
    }

    printf("Fixing: %s => %s\n", inputFileName, outputFileName);

    StopWatch* watch = newStopWatch("Algorithm time");
    ProgressBar* progress = newProgressBar(totalNumberOfPoints, 50, watch);
    draw(progress);
    TrajectoryReader* reader = newTrajectoryReader(input);
    TrajectoryWriter* writer = newTrajectoryWriter(output);

    
    startClock(watch);

    pthread_t pid1, pid2;

    Trajectory* t = readTrajectory(reader);

    while(t != NULL) {

        pthread_create(&pid1, NULL, readTrajectoryOnThread, (void*) reader);
        
        pthread_create(&pid2, NULL, sliceNspliceNsaveOnThread, (void*) newAuxParameter(t, writer, watches));
        // sliceNspliceNsave(t, writer, watches);

        pthread_join(pid2, NULL);

        set(progress, progress->current + t->filled);
        freeTrajectory(t);

        pthread_join(pid1, (void**)(&t));
        // t = readTrajectory(reader);
    }
    stopClock(watch);

    flushProgress(progress);
    printf("\n");

    fclose(input);
    fclose(output);
}


// -----------------------------------------------------------------------------
// -------------------------------   Main   ------------------------------------

int main(int argc, char** argv) {

    printf("max angular speed: %lf\n", MAX_ANGULAR_SPEED);

    if (argc != 2) {
        printf("Invalid number of arguments, expected 2, found %d\n", argc);
        return 1;
    }
    StopWatch* watches[] = {newStopWatch("main_process"), 
                            newStopWatch("sort"), 
                            newStopWatch("actual_slice"), 
                            newStopWatch("write_trajectory"),
                            newStopWatch("get_nearest_point"), 
                            newStopWatch("number_of_points")};
    readAndProcess(argv[1], watches);
    int i;
    for (i = 0; i < 6; i++) {
        printf("%s : %.2lf s\n", watches[i]->name, watches[i]->current);
    }
	return 0;
}
