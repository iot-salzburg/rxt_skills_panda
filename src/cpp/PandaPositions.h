
#include <map>
#include <vector>

//--------------------------------------------------------------------------
// Holds all static positions of panda pathways
//--------------------------------------------------------------------------
class PandaPositions{

public:
    std::vector<double> getPosition(std::string position);

    struct MovementException : public std::exception {
        const char * what () const throw () {
            return "Movement cannot be executed";
        }
    };

    struct GoalToleranceExceededException : public std::exception {
        const char * what () const throw () {
            return "Goal tolerance exceeded";
        }
    };
};
