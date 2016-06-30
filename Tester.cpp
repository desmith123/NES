#include <iostream>
#include "Memory.h"
#include "Processor.h"

int main() {

	std::cout << "DIS THE START OF SUMTIN GREAT" << std::endl;

	Processor *processor = new Processor();
	processor->start(0xC000);

	for (;;);
}