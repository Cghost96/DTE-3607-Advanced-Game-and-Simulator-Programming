//#include <coldet/bits/scenario_fixtures.h>
//#include <coldet/bits/types.h>
//#include <coldet/mysolvers/limited_planes_timer_data.h>
//#include <coldet/mysolvers/limited_planes_data.h>
//#include <coldet/utils/scenario_factories.h>

//#include <iostream>
//#include <stdio.h>
//#include <chrono>
//#include <locale.h>

/**********************************************************************
 *                                                                    *
 *                                                                    *
 *     REMEMBER TO COMMENT OUT MAIN() IN "MAIN.CPP" TO USE THIS       *
 *                                                                    *
 *                                                                    *
 **********************************************************************/

// namespace factory       = dte3607::coldet::utils::scenario_factories;
// namespace lp_data       = dte3607::coldet::mysolvers::limited_planes_data;
// namespace lp_timer_data = dte3607::coldet::mysolvers::limited_planes_timer_data;

// using data_fixture_t
//  = dte3607::coldet::scenario_fixtures::Fixture_Os_Fs_ip_lp_cPsfp_sa_ds_t_counter_distribution;
// using data_fixture =
// dte3607::coldet::scenario_fixtures::Fixture_Os_Fs_ip_lp_cPsfp_sa_ds_counter_distribution; using NS =
// dte3607::coldet::types::NanoSeconds;

// void printxDistribution(std::vector<int> d) {
//  std::cout << "X distribution" << std::endl;
//  std::cout << "index | amount" << std::endl;
//  int total = 0;
//  for (int i = 0; i < d.size(); i++) {
//    std::printf("%d|%d\n", i - 25, d[i]);
//    total += d[i];
//  }
//  std::cout << std::endl;
//  std::cout << "Total: " << total << std::endl;
//}

// void printzDistribution(std::vector<int> d) {
//  std::cout << "Z distribution" << std::endl;
//  std::cout << "index | amount" << std::endl;
//  int total = 0;
//  for (int i = 0; i < d.size(); i++) {
//    std::printf("%d|%d\n", i - 25, d[i]);
//    total += d[i];
//  }
//  std::cout << std::endl;
//  std::cout << "Total: " << total << std::endl;
//}

// int main() {
//  setlocale(LC_NUMERIC, "French_Canada.1252");
//  using namespace std::chrono_literals;

//  auto const timestep = NS(16ms);

//  std::cout << "Starting construction..." << std::endl;
//  /*NO TOUCHING*/
//  std::unique_ptr<data_fixture> scenario01 = factory::constructGaltonData01();
//  std::unique_ptr<data_fixture> scenario02 = factory::constructGaltonData02();
//  std::unique_ptr<data_fixture> scenario03 = factory::constructGaltonData03();
//  std::unique_ptr<data_fixture> scenario04 = factory::constructGaltonData04();

//  /*TOUCHING*/
//  std::unique_ptr<data_fixture> scenario05 = factory::constructGaltonData05();
//  std::unique_ptr<data_fixture> scenario06 = factory::constructGaltonData06();

//  /*TOUCHING AND TIMER*/
//  std::unique_ptr<data_fixture_t> scenario07 = factory::constructGaltonData07();
//  std::unique_ptr<data_fixture_t> scenario08 = factory::constructGaltonData08();
//  std::unique_ptr<data_fixture_t> scenario09 = factory::constructGaltonData09();
//  std::unique_ptr<data_fixture_t> scenario10 = factory::constructGaltonData10();
//  std::unique_ptr<data_fixture_t> scenario11 = factory::constructGaltonData11();
//  std::unique_ptr<data_fixture_t> scenario12 = factory::constructGaltonData12();
//  std::cout << "Finished construction..." << std::endl;

//  std::cout << "--------------------------------------------" << std::endl;
//  std::cout << "--------------NO TOUCHING----------------" << std::endl;
//  std::cout << "--------------------------------------------" << std::endl << std::endl;

//  std::cout << "-------------50 spheres--------------------" << std::endl << std::endl;
//  while (not lp_data::solve(scenario01, timestep))
//    ;
//  printxDistribution(scenario01->xdistribution());
//  printzDistribution(scenario01->zdistribution());

//  std::cout << std::endl << "-------------80 spheres--------------------" << std::endl << std::endl;
//  while (not lp_data::solve(scenario02, timestep))
//    ;
//  printxDistribution(scenario02->xdistribution());
//  printzDistribution(scenario02->zdistribution());

//  std::cout << std::endl << "-------------100 spheres--------------------" << std::endl << std::endl;
//  while (not lp_data::solve(scenario03, timestep))
//    ;
//  printxDistribution(scenario03->xdistribution());
//  printzDistribution(scenario03->zdistribution());

//  std::cout << std::endl << "-------------150 spheres--------------------" << std::endl << std::endl;
//  while (not lp_data::solve(scenario04, timestep))
//    ;
//  printxDistribution(scenario04->xdistribution());
//  printzDistribution(scenario04->zdistribution());


//  std::cout << "--------------------------------------------" << std::endl;
//  std::cout << "--------------TOUCHING----------------" << std::endl;
//  std::cout << "--------------------------------------------" << std::endl << std::endl;

//  std::cout << std::endl << "-------------100 spheres--------------------" << std::endl << std::endl;
//  while (not lp_data::solve(scenario05, timestep))
//    ;
//  printxDistribution(scenario05->xdistribution());
//  printzDistribution(scenario05->zdistribution());

//  std::cout << std::endl << "-------------200 spheres--------------------" << std::endl << std::endl;
//  while (not lp_data::solve(scenario06, timestep))
//    ;
//  printxDistribution(scenario06->xdistribution());
//  printzDistribution(scenario06->zdistribution());


//  std::cout << "--------------------------------------------" << std::endl;
//  std::cout << "--------------TOUCHING AND TIMER----------------" << std::endl;
//  std::cout << "--------------------------------------------" << std::endl << std::endl;

//  std::cout << std::endl << "-------------100 spheres--------------------" << std::endl << std::endl;
//  while (not lp_timer_data::solve(scenario07, timestep))
//    ;
//  printxDistribution(scenario07->xdistribution());
//  printzDistribution(scenario07->zdistribution());

//  std::cout << std::endl << "-------------200 spheres--------------------" << std::endl << std::endl;
//  while (not lp_timer_data::solve(scenario08, timestep))
//    ;
//  printxDistribution(scenario08->xdistribution());
//  printzDistribution(scenario08->zdistribution());

//  std::cout << std::endl << "-------------300 spheres--------------------" << std::endl << std::endl;
//  while (not lp_timer_data::solve(scenario09, timestep))
//    ;
//  printxDistribution(scenario09->xdistribution());
//  printzDistribution(scenario09->zdistribution());

//  std::cout << std::endl << "-------------800 spheres--------------------" << std::endl << std::endl;
//  while (not lp_timer_data::solve(scenario10, timestep))
//    ;
//  printxDistribution(scenario10->xdistribution());
//  printzDistribution(scenario10->zdistribution());

//  std::cout << std::endl
//            << "-------------1500 spheres!!!!! (⊙＿⊙') <- CPU ---------------" << std::endl
//            << std::endl;
//  while (not lp_timer_data::solve(scenario11, timestep))
//    ;
//  printxDistribution(scenario11->xdistribution());
//  printzDistribution(scenario11->zdistribution());
//  std::cout << std::endl << "-------------IT WORKED!!!  ※\\(^o^)/※ <- CPU ----------------" << std::endl;


//  std::cout << std::endl << "----- 3000 spheres!!!!! щ（ﾟДﾟщ) <- CPU ------" << std::endl << std::endl;
//  while (not lp_timer_data::solve(scenario12, timestep))
//    ;
//  printxDistribution(scenario12->xdistribution());
//  printzDistribution(scenario12->zdistribution());
//  std::cout << std::endl << "-----IT WORKED!!! BUT AT WHAT COST?  (×_×) <- CPU----" << std::endl;
//}
