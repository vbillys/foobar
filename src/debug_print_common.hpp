#ifndef DEBUG_PRINT
#define DEBUG_PRINT

#define DEBUG_TEST_0 1
#define DEBUG_TEST_1 1
#define DEBUG_TEST_2 1
#define DEBUG_TEST_3 1
#define DEBUG_TEST_4 1
#define DEBUG_TEST_5 1
#define DEBUG_TEST_6 1

class GlobalDebugData{
	public:
	    static bool getDebugTest0Flag(){return g_debug_test_0;}
	    static bool getDebugTest1Flag(){return g_debug_test_1;}
	    static bool getDebugTest2Flag(){return g_debug_test_2;}
	    static bool getDebugTest3Flag(){return g_debug_test_3;}
	    static bool getDebugTest4Flag(){return g_debug_test_4;}
	    static bool getDebugTest5Flag(){return g_debug_test_5;}
	    static bool getDebugTest6Flag(){return g_debug_test_6;}
	    
	    static void setDebugTest0Flag(bool flag){g_debug_test_0 = flag;}
	    static void setDebugTest1Flag(bool flag){g_debug_test_1 = flag;}
	    static void setDebugTest2Flag(bool flag){g_debug_test_2 = flag;}
	    static void setDebugTest3Flag(bool flag){g_debug_test_3 = flag;}
	    static void setDebugTest4Flag(bool flag){g_debug_test_4 = flag;}
	    static void setDebugTest5Flag(bool flag){g_debug_test_5 = flag;}
	    static void setDebugTest6Flag(bool flag){g_debug_test_6 = flag;}
	private:
		static bool g_debug_test_0;// = DEBUG_TEST_0;
		static bool g_debug_test_1;// = DEBUG_TEST_1;
		static bool g_debug_test_2;// = DEBUG_TEST_2;
		static bool g_debug_test_3;// = DEBUG_TEST_3;
		static bool g_debug_test_4;// = DEBUG_TEST_4;
		static bool g_debug_test_5;// = DEBUG_TEST_5;
		static bool g_debug_test_6;// = DEBUG_TEST_6;
};


#define debug_print_0( ...) \
            do { if (GlobalDebugData::getDebugTest0Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)

#define debug_print_1( ...) \
            do { if (GlobalDebugData::getDebugTest1Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)
            
#define debug_print_2( ...) \
            do { if (GlobalDebugData::getDebugTest2Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)
            
#define debug_print_3( ...) \
            do { if (GlobalDebugData::getDebugTest3Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)

#define debug_print_4( ...) \
            do { if (GlobalDebugData::getDebugTest4Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)

#define debug_print_5( ...) \
            do { if (GlobalDebugData::getDebugTest5Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)

#define debug_print_6( ...) \
            do { if (GlobalDebugData::getDebugTest6Flag()) fprintf(stdout,  __VA_ARGS__); } while (0)

#endif
