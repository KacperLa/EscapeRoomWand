#ifndef GESTURE_RECOGNIZER_H
#define GESTURE_RECOGNIZER_H

#include <Arduino.h>
#include <math.h>

struct Pos
{
  int x;
  int y;

  Pos() { x = 0; y = 0; };
  Pos(int new_x, int new_y) {x = new_x; y = new_y; };

  bool operator==(const Pos& other) const {
    return (x == other.x) && (y == other.y);
  }
};

static const int MAX_POINTS = 64; // Adjust as needed
static const int MAX_CHECKPOINTS = 6; // Adjust as needed 


class GestureRecognizer {
public:
    // Constructor:
    //   points: 2D array of [x,y] for the spell shape
    //   n:      number of points in the spell
    //   thresh: distance threshold for considering a checkpoint "hit"
    GestureRecognizer(const Pos points[], int n, float thresh)
    {
        threshold = thresh;
        totalPoints = (n < MAX_POINTS) ? n : MAX_POINTS;
        for(int i = 0; i < totalPoints; i++){
            spellPoints[i] = points[i];
        }
        resetSpellProgress();
    }

    // Resets progress toward casting the spell
    void resetSpellProgress()
    {
        prog = -1;             // which checkpoint we are on
        started = false;      // have we started the gesture?
        checkpointOffset = 0; // offset for the local position mapping
    }

    // Checks the current wand-tip position and updates progress.
    //   posX, posY: current wand-tip coordinates
    //   detected:   whether the wand is "detected" at all
    //   results[]:  an output array of bools that will be set true up to the last checkpoint reached.
    //
    //   The caller should provide a bool array of at least 'totalPoints' length.
    void checkSpellProgress(Pos pos, bool detected, bool results[])
    {
        // If the wand is not detected, reset everything
        if(!detected) {
            if(started) {
                // If we had started, we treat losing detection as a reset
                resetSpellProgress();
            }
        }
        else {
            // If not started yet and the wand is steady
            if(!started) {
                started = true;
            }

            // If we have started tracking
            if(started && prog < (totalPoints - 1)) {
                // next checkpoint
                int nextIdx = prog + 1;
                Pos nextPos = spellPoints[nextIdx];

                // Distance from local pos to next checkpoint
                float dx = pos.x - nextPos.x;
                float dy = pos.y - nextPos.y;
                float distCheckpoint = sqrtf(dx*dx + dy*dy);

                // If we are close enough to the next checkpoint, increment progress
                if(distCheckpoint < threshold) {
                    prog++;
                }
            }

            if(prog >= (totalPoints - 1)) {
                // Clamp progress at the last checkpoint
                prog = totalPoints - 1;
            }
        }

        // Finally, fill in the results array (True up to current progress)
        for(int i = 0; i < totalPoints; i++){
            // "i <= prog" means checkpoint i has been "hit"
            results[i] = (i <= prog);
        }
    }

private:

    Pos spellPoints[MAX_POINTS];
    int   totalPoints;
    float threshold;

    // Tracking variables
    int   prog;            // which checkpoint is next
    bool  started;         // have we "started"
    int   checkpointOffset;
};

class GestureHistory {
public:
    // Constructor:
    //   points: 2D array of [x,y] for the spell shape
    //   n:      number of points in the spell
    //   thresh: distance threshold for considering a checkpoint "hit"
    GestureHistory(const Pos points[], int n, float thresh)
    {
        threshold = thresh;
        totalPoints = (n < MAX_POINTS) ? n : MAX_POINTS;
        for(int i = 0; i < totalPoints; i++){
            spellPoints[i] = points[i];
        }

        // Determne that no two points are with in the threshold
        for(int i = 0; i < totalPoints; i++){
            for(int j = i + 1; j < totalPoints; j++) {
                float dx = spellPoints[i].x - spellPoints[j].x;
                float dy = spellPoints[i].y - spellPoints[j].y;
                float dist = sqrtf(dx*dx + dy*dy);
                if(dist < threshold) {
                    // If two points are too close, we cannot use this gesture
                    totalPoints = 0;
                    Serial.println("Error: Two points are too close in the gesture definition.");
                    return;
                }
            }
        }

        resetSpellHistory();
    }

    // Resets progress toward casting the spell
    void resetSpellHistory()
    {
        started = false;      // have we started the gesture?
        memset(checkpointHistory, -1, sizeof(checkpointHistory));
    }

    void pushCheckpoint(int idx)
    {
        // If we have not started, we cannot push checkpoints
        if(!started) {
            return;
        }

        if (idx == checkpointHistory[0]) {
            // If the checkpoint is already at the front, do nothing
            return;
        }

        // Shift the history to make room for the new checkpoint (shift right)
        for(int i = MAX_CHECKPOINTS - 1; i > 0; i--) {
            checkpointHistory[i] = checkpointHistory[i-1];
        }

        checkpointHistory[0] = idx;
    }

    int getCheckpointHistoryAtIndex(int index)
    {
        // Get the checkpoint history at the given index
        // Returns -1 if index is out of bounds
        if(index < 0 || index >= MAX_CHECKPOINTS) {
            return -1;
        }
        return checkpointHistory[index];
    }

    void getCheckpointHistory(int history[], int size)
    {
        if (size <= 0 || size > MAX_CHECKPOINTS) {
            Serial.println("Error: Invalid size for checkpoint history.");
            return;
        }
        
        // Copy the checkpoint history to the provided array
        memcpy(history, checkpointHistory, size * sizeof(int));
    }

    // Checks the current wand-tip position and updates progress.
    //   posX, posY: current wand-tip coordinates
    //   detected:   whether the wand is "detected" at all
    //
    void checkSpellProgress(Pos pos, bool detected)
    {
        // If the wand is not detected, reset everything
        if(!detected) {
            if(started) {
                // If we had started, we treat losing detection as a reset
                resetSpellHistory();
            }
        }
        else {
            // If not started yet and the wand is steady
            if(!started) {
                started = true;
            }

            // Iterate through the checkpoints
            // Determine is we are with in range of any checkpoint
            for(int i = 0; i < totalPoints; i++) {
                float dx = spellPoints[i].x - pos.x;
                float dy = spellPoints[i].y - pos.y;
                float dist = sqrtf(dx*dx + dy*dy);
                if(dist < threshold) {
                    // If pos is within the threshold of a checkpoint
                    pushCheckpoint(i);
                    return;
                }
            }
        }
    }

private:
    Pos spellPoints[MAX_POINTS];
    int   totalPoints;
    float threshold;

    // Tracking variables
    bool  started;         // have we "started"
    // History of Checkpoints array of ids
    int checkpointHistory[MAX_CHECKPOINTS];
};

#endif // GESTURE_RECOGNIZER_H
