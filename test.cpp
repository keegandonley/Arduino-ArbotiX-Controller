#include <iostream>
#include <math.h>

#define PI 3.1415926535

int main() {
	int x = 12;
	int y = 12;
	int z = 12;

	int A = 15;
	int B = 15;
	int C = 8;
	int D = 4;

	int theta1 = atan(y / x) * 180 / PI;
	int xc = x - cos(theta1) * D;
	int yc = y - sin(theta1) * D;
	int d = ((pow(xc, 2) + pow(yc, 2) + pow((z - A), 2) - pow(B, 2) - pow(C, 2)) / (2 * B * C));
	int theta2 = (atan((z - A) / sqrt(pow(xc, 2) + pow(yc, 2)))) * 180 / PI;
	int theta3 = (atan(sqrt(1 - pow(d, 2)) / d)) * 180 / PI;
	int theta4 = (theta2 * -1 + theta3);
	int theta5 = 0;

	theta1 = 30;
	theta2 = 60;
	theta3 = 20;
	theta4 = 15;
	theta5 = 22;

	int jointpos1 = theta1 * 1024 / 60;
  	int jointpos2 = theta2 * 1024 / 60;
  	int jointpos3 = theta3 * 1024 / 60;
  	int jointpos4 = theta4 * 1024 / 60;
  	int jointpos5 = theta5 * 1024 / 60;

	std::cout << "D: " << d << std::endl;
	std::cout << "Theta 1: " << theta1 << std::endl;
	std::cout << "Theta 2: " << theta2 << std::endl;
	std::cout << "Theta 3: " << theta3 << std::endl;
	std::cout << "Theta 4: " << theta4 << std::endl;
	std::cout << "Theta 5: " << theta5 << std::endl;

	std::cout << "jointpos 1: " << jointpos1 << std::endl;
	std::cout << "jointpos 2: " << jointpos2 << std::endl;
	std::cout << "jointpos 3: " << jointpos3 << std::endl;
	std::cout << "jointpos 4: " << jointpos4 << std::endl;
	std::cout << "jointpos 5: " << jointpos5 << std::endl;
}
