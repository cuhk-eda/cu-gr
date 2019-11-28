#include "Setting.h"
#include "Database.h"

namespace db {

void Setting::makeItSilent() {
    multiNetVerbose = VerboseLevelT::LOW;
    dbVerbose = VerboseLevelT::LOW;
}

void Setting::adapt() {
    if (database.nets.size() > 800000) {
         --rrrIterLimit;
    
    }
}

Setting setting;

}  // namespace db
