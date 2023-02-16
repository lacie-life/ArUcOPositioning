#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <string>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>



using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define KEY_TOW 87
#define KEY_FOR 83
#define KEY_LEFT 65
#define KEY_RIGHT 68
#define KEY_X 81

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {

    //DO whatever you like to do with this charecter ..

    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

void component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using NED co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_ctrl_ned(std::shared_ptr<mavsdk::Offboard> offboard)
{
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    offboard_log(offb_mode, "Turn to face East");
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 90.0f});
    sleep_for(seconds(1)); // Let yaw settle.

    {
        const float step_size = 0.01f;
        const float one_cycle = 2.0f * (float)M_PI;
        const unsigned steps = 2 * unsigned(one_cycle / step_size);

        offboard_log(offb_mode, "Go North and back South");
        for (unsigned i = 0; i < steps; ++i) {
            float vx = 5.0f * sinf(i * step_size);
            offboard->set_velocity_ned({vx, 0.0f, 0.0f, 90.0f});
            sleep_for(milliseconds(10));
        }
    }

    offboard_log(offb_mode, "Turn to face West");
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 270.0f});
    sleep_for(seconds(2));

    offboard_log(offb_mode, "Go up 2 m/s, turn to face South");
    offboard->set_velocity_ned({0.0f, 0.0f, -2.0f, 180.0f});
    sleep_for(seconds(4));

    offboard_log(offb_mode, "Go down 1 m/s, turn to face North");
    offboard->set_velocity_ned({0.0f, 0.0f, 1.0f, 0.0f});
    sleep_for(seconds(4));

    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}

/**
 * Does Offboard control using body co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_ctrl_body(std::shared_ptr<mavsdk::Offboard> offboard)
{
    const std::string offb_mode = "BODY";

    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed: ");
    offboard_log(offb_mode, "Offboard started");

    offboard_log(offb_mode, "Turn clock-wise and climb");
    offboard->set_velocity_body({0.0f, 0.0f, -1.0f, 60.0f});
    sleep_for(seconds(5));

    offboard_log(offb_mode, "Turn back anti-clockwise");
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, -60.0f});
    sleep_for(seconds(5));

    offboard_log(offb_mode, "Wait for a bit");
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(2));

    offboard_log(offb_mode, "Fly a circle");
    offboard->set_velocity_body({5.0f, 0.0f, 0.0f, 30.0f});
    sleep_for(seconds(15));

    offboard_log(offb_mode, "Wait for a bit");
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(5));

    offboard_log(offb_mode, "Fly a circle sideways");
    offboard->set_velocity_body({0.0f, -5.0f, 0.0f, 30.0f});
    sleep_for(seconds(15));

    offboard_log(offb_mode, "Wait for a bit");
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(8));

    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}

/**
 * Does Offboard control using attitude commands.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_ctrl_attitude(std::shared_ptr<mavsdk::Offboard> offboard)
{
    const std::string offb_mode = "ATTITUDE";

    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_attitude({30.0f, 0.0f, 0.0f, 0.6f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    offboard_log(offb_mode, "ROLL 30");
    offboard->set_attitude({30.0f, 0.0f, 0.0f, 0.6f});
    sleep_for(seconds(2)); // rolling

    offboard_log(offb_mode, "ROLL -30");
    offboard->set_attitude({-30.0f, 0.0f, 0.0f, 0.6f});
    sleep_for(seconds(2)); // Let yaw settle.

    offboard_log(offb_mode, "ROLL 0");
    offboard->set_attitude({0.0f, 0.0f, 0.0f, 0.6f});
    sleep_for(seconds(2)); // Let yaw settle.

    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}

// Pition ned
bool offb_ctrl_position_ned(std::shared_ptr<mavsdk::Offboard> offboard)
{
    const std::string offb_mode = "POSITION_NED";
    offboard->set_position_ned({0.0f, 0.0f, -1.5f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    offboard_log(offb_mode, "1 0 1.5 0");
    offboard->set_position_ned({1.0f, 0.0f, -1.5f, 0.0f});
    sleep_for(seconds(5));

    offboard_log(offb_mode, "1 1 1.5 0");
    offboard->set_position_ned({1.0f, 1.0f, -1.5f, 0.0f});
    sleep_for(seconds(5));

    offboard_log(offb_mode, "2 2 1.5 0");
    offboard->set_position_ned({2.0f, 2.0f, -1.5f, 0.0f});
    sleep_for(seconds(5)); 

    offboard_log(offb_mode, "0 0 1.5 0");
    offboard->set_position_ned({0.0f, 0.0f, -1.5f, 0.0f});
    sleep_for(seconds(5)); 

    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}

// Keyboard control
bool keyboard_ctrl(std::shared_ptr<mavsdk::Offboard> offboard)
{
    // Control via keyboard
    // UP : W
    // DOWN : S
    // LEFT : A
    // RIGHT : D
    // Land : Q
    std::cout << "[Start control via keyboard]" << std::endl;
    char key;
    int value;

    const std::string offb_mode = "Keyboard";
    offboard->set_position_ned({0.0f, 0.0f, -1.5f, 0.0f});
    float north = 0.0f;
    float east = 0.0f;
    float down = -1.5f;

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");
    
    while(1){

        if(kbhit()){
            key = getch();
            value = key;

            while(value != KEY_X)
            {
                switch (getch())
                {
                    case KEY_TOW:
                    {
                        offboard_log(offb_mode, "TOWARD");
                        east = east+0.1f;
                      //  offboard->set_position_ned({1.0f, 0.0f, -1.5f, 0.0f});
                      //  sleep_for(seconds(5));
                        offboard->set_position_ned({north, east, down, 0.0f});
                        break;
                    }
                    case KEY_FOR:
                    {
                        offboard_log(offb_mode, "FORWARD");
                        east = east-0.1f;
                        offboard->set_position_ned({north, east, down, 0.0f});
                        break;
                    }
                    case KEY_LEFT:
                    {
                        offboard_log(offb_mode, "LEFT");
                        north = north+0.1f;
                        offboard->set_position_ned({north, east, down, 0.0f});
                        break;
                    }
                    case KEY_RIGHT:
                    {
                        offboard_log(offb_mode, "RIGHT");
                        north = north-0.1f;
                        offboard->set_position_ned({north, east, down, 0.0f});
                        break;
                    }
                }
                key = getch();
                value = key;
            }
            return true;
        }
    }
    return false;
}


int main(int argc, char** argv)
{
    Mavsdk dc;
    std::string connection_url;
    ConnectionResult connection_result;

    bool discovered_system = false;
    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // We don't need to specify the UUID if it's only one system anyway.
    // If there were multiple, we could specify it with:
    // dc.system(uint64_t uuid);
    System& system = dc.system();

    std::cout << "Waiting to discover system..." << std::endl;
    dc.register_on_discover([&discovered_system](uint64_t uuid) {
        std::cout << "Discovered system with UUID: " << uuid << std::endl;
        discovered_system = true;
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));

    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // Register a callback so we get told when components (camera, gimbal) etc
    // are found.
    system.register_component_discovered_callback(component_discovered);

    auto telemetry = std::make_shared<Telemetry>(system);
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);


    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Setting rate failed:" << Telemetry::result_str(set_rate_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->position_async([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
    });
    std::cout << telemetry->rc_status() << std::endl;

   // Check if vehicle is ready to arm
    while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        std::cout << telemetry->health() << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();

    if (arm_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << Action::result_str(arm_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Set take off altitude
    std::cout << "Set take off altitude is 1.5 m " << std::endl;
    const Action::Result set_takeoff_altitude_result = action->set_takeoff_altitude(1.5);
    if(set_takeoff_altitude_result != Action::Result::SUCCESS){
        std::cout << ERROR_CONSOLE_TEXT << "Set takeoff altitude failed:" << Action::result_str(set_takeoff_altitude_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << Action::result_str(takeoff_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));

    mavsdk::Telemetry::Position postion = telemetry->position();
    std::cout << "Global: " << postion << std::endl;

    mavsdk::Telemetry::PositionNED positionNED = (telemetry->position_velocity_ned()).position;
    std::cout << "Local: " << positionNED.down_m << " "
                           << positionNED.east_m << " " 
                           << positionNED.north_m << std::endl;

    bool ret;

    // //  using attitude control
    // bool ret = offb_ctrl_attitude(offboard);
    // if (ret == false) {
    //     return EXIT_FAILURE;
    // }

    // //  using local NED co-ordinates
    // ret = offb_ctrl_ned(offboard);
    // if (ret == false) {
    //     return EXIT_FAILURE;
    // }

    // //  using body co-ordinates
    // ret = offb_ctrl_body(offboard);
    // if (ret == false) {
    //     return EXIT_FAILURE;
    // }

    // using position ned co-ordinates
    ret = offb_ctrl_position_ned(offboard);
    if (ret == false) {
        return EXIT_FAILURE;
    }

    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));

    ret = keyboard_ctrl(offboard);
    if(ret == false) {
        return EXIT_FAILURE;
    }

    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << Action::result_str(land_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return 0;
}