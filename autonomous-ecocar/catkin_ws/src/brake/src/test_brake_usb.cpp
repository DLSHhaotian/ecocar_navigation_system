#include "JrkG2.h"
#include <iostream>

int main(){
	JrkG2 conn = JrkG2();
	conn.connect();
	if(conn.setTarget(42)){
		std::cout << " ok" << std::endl;
	}
}
