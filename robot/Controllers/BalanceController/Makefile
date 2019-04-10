
testBalanceController: testBalanceController.cpp 
	g++ -O2 -o testBalanceController testBalanceController.cpp \
	 MyThreadClass.cpp OptimizationThread.cpp BalanceController.cpp \
	-I$(CHEETAH_SOFTWARE_PATH)/LCM-Types \
	-I$(CHEETAH_SOFTWARE_PATH)/Third-Party/eigen \
	-I$(CHEETAH_SOFTWARE_PATH)/Third-Party/qpOASES-3.2.0/include \
	-L$(CHEETAH_SOFTWARE_PATH)/Third-Party/qpOASES-3.2.0/bin \
	-Wl,-rpath,$(CHEETAH_SOFTWARE_PATH)/Third-Party/qpOASES-3.2.0/bin \
	-lqpOASES -pthread -lpthread -llcm -std=c++11 

clean:
	rm testBalanceController

