#define numSteppers 5

class moveElement {

    public:

        float position[numSteppers]; //Positon the Robotarm will move to
        int speed; // The Speed at which the arm will move to the Position
        int delay; // The delay AFTER the robotarm has moved (in ms) in this time no other things can be done
        char flag; // Usefull flags like Pause etc.
        bool written; // If element has been used

    moveElement(){
        written = false;
    }

    void set(float pos[numSteppers], int s = 400, int d = 0, char f= '0') {
        for (int i = 0; i<numSteppers; i++) {
            position[i] = pos[i];
        }
        speed = s;
        delay = d;
        flag = f;
        written = true;
    }

    void reset() {
        written = false;
    }
    
};