#include "db/Database.h"
#include "global.h"
#include "multi_net/Router.h"
#include "gr_db/GrDatabase.h"

double LARGE_NUM = 100000000;

// -----------------------------------------------------------------------------

void signalHandler(int signum) {
    std::cout << "Signal (" << signum << ") received. Exiting...\n";

    // cleanup and close up stuff here

    std::exit(signum);
}

// -----------------------------------------------------------------------------

void runISPD18Flow(const boost::program_options::variables_map& vm) {
    // db::setting.makeItSilent();

    Rsyn::Session session;

    // Parse options
    // required
    std::string lefFile = vm.at("lef").as<std::string>();
    std::string defFile = vm.at("def").as<std::string>();
    db::setting.numThreads = vm.at("threads").as<int>();
    db::setting.outputFile = vm.at("output").as<std::string>();
    // optional
    if (vm.count("tat")) {
        db::setting.tat = vm.at("tat").as<int>();
    }
    // multi_net
    if (vm.count("multiNetVerbose")) {
        db::setting.multiNetVerbose =
            db::VerboseLevelT::_from_string(vm.at("multiNetVerbose").as<std::string>().c_str());
    }
    if (vm.count("multiNetScheduleSortAll")) {
        db::setting.multiNetScheduleSortAll = vm.at("multiNetScheduleSortAll").as<bool>();
    }
    if (vm.count("multiNetScheduleReverse")) {
        db::setting.multiNetScheduleReverse = vm.at("multiNetScheduleReverse").as<bool>();
    }
    if (vm.count("multiNetScheduleSort")) {
        db::setting.multiNetScheduleSort = vm.at("multiNetScheduleSort").as<bool>();
    }
    if (vm.count("rrrIters")) {
        db::setting.rrrIterLimit = vm.at("rrrIters").as<int>();
    }
    if (vm.count("rrrWriteEachIter")) {
        db::setting.rrrWriteEachIter = vm.at("rrrWriteEachIter").as<bool>();
    }
    if (vm.count("rrrInitVioCostDiscount")) {
        db::setting.rrrInitVioCostDiscount = vm.at("rrrInitVioCostDiscount").as<double>();
    }
    if (vm.count("rrrFadeCoeff")) {
        db::setting.rrrFadeCoeff = vm.at("rrrFadeCoeff").as<double>();
    }
    if (vm.count("edgeShiftingIter")) {
        db::setting.edgeShiftingIter = vm.at("edgeShiftingIter").as<int>();
    }
    // single_net
    if (vm.count("fixOpenBySST")) {
        db::setting.fixOpenBySST = vm.at("fixOpenBySST").as<bool>();
    }
    // db
    if (vm.count("dbVerbose")) {
        db::setting.dbVerbose = db::VerboseLevelT::_from_string(vm.at("dbVerbose").as<std::string>().c_str());
    }
    if (vm.count("dbInitHistUsageForPinAccess")) {
        db::setting.dbInitHistUsageForPinAccess = vm.at("dbInitHistUsageForPinAccess").as<double>();
    }

    vector<std ::string> strs;
    boost::split(strs, db::setting.outputFile, boost::is_any_of("."));
    db::setting.name = strs[0];

    // Read benchmarks
    Rsyn::ISPD2018Reader reader;
    const Rsyn::Json params = {
        {"lefFile", lefFile},
        {"defFile", defFile},
    };
    log() << std::endl;
    if (db::setting.dbVerbose >= +db::VerboseLevelT::HIGH) {
        log() << "################################################################" << std::endl;
        log() << "Start reading benchmarks" << std::endl;
    }
    reader.load(params);
    if (db::setting.dbVerbose >= +db::VerboseLevelT::HIGH) {
        log() << "Finish reading benchmarks" << std::endl;
        log() << "MEM: cur=" << utils::mem_use::get_current() << "MB, peak=" << utils::mem_use::get_peak() << "MB"
              << std::endl;
        log() << std::endl;
    }

    // Route
    database.init();
    db::setting.adapt();
    grDatabase.init();

    log() << "finish reading benchmark" << std::endl;

    Router router;
    router.run();

    grDatabase.writeGuides(db::setting.outputFile);

    database.clear();
    grDatabase.clear();

    log() << "MEM: cur=" << utils::mem_use::get_current() << "MB, peak=" << utils::mem_use::get_peak() << "MB"
          << std::endl;
    log() << std::endl;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    Rsyn::Session::init();

    printlog("------------------------------------------------------------------------------");
    printlog("                     ICCAD 2019 - Global Routing Contest                      ");
    printlog("                             Team number : 15                                 ");
    printlog("                             Team name : CU-GR                                ");
    printlog("        Members: Jinwei Liu, Chak-Wa Pui, Fangzhou Wang,                      ");
    printlog("                 Evangeline F.Y. Young                                        ");
    printlog("        Affiliation: The Chinese University of Hong Kong                      ");
    printlog("------------------------------------------------------------------------------");

    std::cout << std::boolalpha;  // set std::boolalpha to std::cout

    try {
        using namespace boost::program_options;
        options_description desc{"Options"};
        // clang-format off
        desc.add_options()
                ("help", "Help message.")
                ("lef", value<std::string>()->required(), "Input .lef file")
                ("def", value<std::string>()->required(), "Input .def file.")
                ("threads", value<int>()->required(), "# of threads")
                ("output", value<std::string>()->required(), "Output file name")
                // optional
                ("tat", value<int>(), "Runtime limit (sec)")
                ("multiNetVerbose", value<std::string>())
                ("multiNetScheduleSortAll", value<bool>())
                ("multiNetScheduleReverse", value<bool>())
                ("multiNetScheduleSort", value<bool>())
                ("rrrIters", value<int>())
                ("rrrWriteEachIter", value<bool>())
                ("rrrInitVioCostDiscount", value<double>())
                ("rrrFadeCoeff", value<double>())
                ("edgeShiftingIter", value<int>())
                ("fixOpenBySST", value<bool>())
                ("dbVerbose", value<std::string>())
                ("dbInitHistUsageForPinAccess", value<double>())
                ;
        // clang-format on
        variables_map vm;
        store(command_line_parser(argc, argv)
                  .options(desc)
                  .style(command_line_style::unix_style | command_line_style::allow_long_disguise)
                  .run(),
              vm);
        notify(vm);
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        for (const auto& option : desc.options()) {
            if (vm.count(option->long_name())) {
                std::string name = option->description().empty() ? option->long_name() : option->description();
                log() << std::left << std::setw(18) << name << ": ";
                const auto& value = vm.at(option->long_name()).value();
                if (auto v = boost::any_cast<double>(&value)) {
                    std::cout << *v;
                } else if (auto v = boost::any_cast<int>(&value)) {
                    std::cout << *v;
                } else if (auto v = boost::any_cast<std::string>(&value)) {
                    std::cout << *v;
                } else if (auto v = boost::any_cast<bool>(&value)) {
                    std::cout << *v;
                } else {
                    std::cout << "unresolved type";
                }
                std::cout << std::endl;
            }
        }
        runISPD18Flow(vm);
    } catch (const boost::program_options::error& e) {
        printlog(e.what());
    }

    printlog("---------------------------------------------------------------------------");
    printlog("                               Terminated...                               ");
    printlog("---------------------------------------------------------------------------");

    return 0;
}
