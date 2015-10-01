#ifndef DEBUG_PRINT_CURB_DETECT
#define DEBUG_PRINT_CURB_DETECT

#include <ros/ros.h>
#include <ros/console.h>

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



namespace globaldebugdata{
    struct BasicFilter : public ros::console::FilterBase
    {
		
    typedef boost::shared_ptr<BasicFilter> Ptr;
		
    BasicFilter(bool enabled)
    : enabled_(enabled)
    {}
    
 
    inline virtual bool isEnabled() { return enabled_; };
 
    bool enabled_;
    public:
        void setFlag(bool is){
			enabled_ = is;
		}
    };
    class DebugFlags{
	    public:
	        static void setFlag0(bool is){g_debug_test_0->setFlag(is);};
	        static void setFlag1(bool is){g_debug_test_1->setFlag(is);};
	        static void setFlag2(bool is){g_debug_test_2->setFlag(is);};
	        static void setFlag3(bool is){g_debug_test_3->setFlag(is);};
	        static void setFlag4(bool is){g_debug_test_4->setFlag(is);};
	        static void setFlag5(bool is){g_debug_test_5->setFlag(is);};
	        static void setFlag6(bool is){g_debug_test_6->setFlag(is);};
	        static BasicFilter::Ptr getFlag0(){return  g_debug_test_0;};
	        static BasicFilter::Ptr getFlag1(){return  g_debug_test_1;};
	        static BasicFilter::Ptr getFlag2(){return  g_debug_test_2;};
	        static BasicFilter::Ptr getFlag3(){return  g_debug_test_3;};
	        static BasicFilter::Ptr getFlag4(){return  g_debug_test_4;};
	        static BasicFilter::Ptr getFlag5(){return  g_debug_test_5;};
	        static BasicFilter::Ptr getFlag6(){return  g_debug_test_6;};
	    private:
	        static BasicFilter::Ptr g_debug_test_0;
	        static BasicFilter::Ptr g_debug_test_1;
	        static BasicFilter::Ptr g_debug_test_2;
	        static BasicFilter::Ptr g_debug_test_3;
	        static BasicFilter::Ptr g_debug_test_4;
	        static BasicFilter::Ptr g_debug_test_5;
	        static BasicFilter::Ptr g_debug_test_6;
	        
    };
    
}
//#define debug_print_0(fmt, ...) \
            //do { if (DEBUG_TEST_0) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

//#define debug_print_1(fmt, ...) \
            //do { if (DEBUG_TEST_1) fprintf(stderr, fmt, __VA_ARGS__); } while (0)
            
//#define debug_print_2(fmt, ...) \
            //do { if (DEBUG_TEST_2) fprintf(stderr, fmt, __VA_ARGS__); } while (0)
            
//#define debug_print_3(fmt, ...) \
            //do { if (DEBUG_TEST_3) fprintf(stderr, fmt, __VA_ARGS__); } while (0)


//#define debug_print_0( ...) \
            //do { if (g_debug_test_0) fprintf(stderr,  __VA_ARGS__); } while (0)

//#define debug_print_1( ...) \
            //do { if (g_debug_test_1) fprintf(stderr,  __VA_ARGS__); } while (0)
            
//#define debug_print_2( ...) \
            //do { if (g_debug_test_2) fprintf(stderr,  __VA_ARGS__); } while (0)
            
//#define debug_print_3( ...) \
            //do { if (g_debug_test_3) fprintf(stderr,  __VA_ARGS__); } while (0)

//#define debug_print_4( ...) \
            //do { if (g_debug_test_4) fprintf(stderr,  __VA_ARGS__); } while (0)

//#define debug_print_5( ...) \
            //do { if (g_debug_test_5) fprintf(stderr,  __VA_ARGS__); } while (0)

//#define debug_print_6( ...) \
            //do { if (g_debug_test_6) fprintf(stderr,  __VA_ARGS__); } while (0)

//#define debug_print_0( ...) \
            //do { if (g_debug_test_0) fprintf(stdout,  __VA_ARGS__); } while (0)

//#define debug_print_1( ...) \
            //do { if (g_debug_test_1) fprintf(stdout,  __VA_ARGS__); } while (0)
            
//#define debug_print_2( ...) \
            //do { if (g_debug_test_2) fprintf(stdout,  __VA_ARGS__); } while (0)
            
//#define debug_print_3( ...) \
            //do { if (g_debug_test_3) fprintf(stdout,  __VA_ARGS__); } while (0)

//#define debug_print_4( ...) \
            //do { if (g_debug_test_4) fprintf(stdout,  __VA_ARGS__); } while (0)

//#define debug_print_5( ...) \
            //do { if (g_debug_test_5) fprintf(stdout,  __VA_ARGS__); } while (0)

//#define debug_print_6( ...) \
            //do { if (g_debug_test_6) fprintf(stdout,  __VA_ARGS__); } while (0)

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
