#include <main.hpp>
#include <signal.h>

void termination_handler(int signum);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tld_tracker_node");
	ros::AsyncSpinner spinner(1);
	struct sigaction action;
	Main * main_node = new Main();

	/* Set up the structure to specify the action */
	action.sa_handler = termination_handler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = 0;
	sigaction(SIGINT, &action, NULL);

	spinner.start();
	main_node->process();
	spinner.stop();

	delete main_node;

	return 0;
}

void termination_handler(int signum)
{
	ros::shutdown();
}

