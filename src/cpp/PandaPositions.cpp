
#include "PandaPositions.h"
#include "ros/ros.h"

//--------------------------------------------------------------------------
// getPosition function
// returns statically trained positions vector
//--------------------------------------------------------------------------
std::vector<double> PandaPositions::getPosition(std::string position){

        ROS_INFO(">>>> goal position [%s] <<<<", position.c_str());
        
        // Map to store coordinates - Key = description as string, Value = vector with double value of the joint positions
        std::map<std::string, std::vector<double>> positions;

        // Position above printer
        positions["initial position"] =
        {
                +1.498139,		 // Joint 1 
                -0.520210,		 // Joint 2 
                +0.097301,		 // Joint 3 
                -2.814416,		 // Joint 4 
                +0.052662,		 // Joint 5 
                +2.351891,		 // Joint 6 
                +0.801563		 // Joint 7 
        };

        // Position above printer
        positions["near printer"] =
        {
                +2.496836,		 // Joint 1 
                +0.772564,		 // Joint 2 
                +1.106343,		 // Joint 3 
                -1.478179,		 // Joint 4 
                -0.645623,		 // Joint 5 
                +1.874790,		 // Joint 6 
                -0.199039		 // Joint 7 
        };

        // Position at printer
        positions["printer"] =
        {
                +2.605499,		// Joint 1
                +1.441675,		// Joint 2
                +1.236548,		// Joint 3
                -1.397383,		// Joint 4
                -0.777287,		// Joint 5
                +2.323873,		// Joint 6
                -0.287551		// Joint 7
        };

        // Position 50cm above conveyor belt
        positions["near conveyor belt"] =
        {
                +0.844795,		 // Joint 1 
                -0.384217,		 // Joint 2 
                +1.890267,		 // Joint 3 
                -1.501299,		 // Joint 4 
                +0.146468,		 // Joint 5 
                +1.830871,		 // Joint 6 
                +2.016475		 // Joint 7 
        };

        // Position at conveyor belt
        positions["conveyor belt"] =
        {
                +0.972066,		// Joint 0
                -1.419312,		// Joint 1
                +1.952559,		// Joint 2
                -1.774845,		// Joint 3
                +0.277226,		// Joint 4
                +2.372050,		// Joint 5
                +2.414976		// Joint 6
        };

        // Position at storage
        positions["storage"] =
        {
                +2.031532,		 // Joint 1 
                +1.054223,		 // Joint 2 
                +2.168870,		 // Joint 3 
                -2.671940,		 // Joint 4 
                -0.792631,		 // Joint 5 
                +3.065037,		 // Joint 6 
                +0.802607		 // Joint 7 
        };

        // Position near storage place 1
        positions["near storage place 1"] =
        {
                +1.900263,		 // Joint 1 
                +0.559978,		 // Joint 2 
                +2.802849,		 // Joint 3 
                -2.067452,		 // Joint 4 
                -0.311117,		 // Joint 5 
                +2.489097,		 // Joint 6 
                +0.959915		 // Joint 7 
        };

        // Position near storage place 1
        positions["storage place 1"] =
        {
                +1.902461,		 // Joint 1 
                +0.410919,		 // Joint 2 
                +2.765295,		 // Joint 3 
                -2.040703,		 // Joint 4 
                -0.174810,		 // Joint 5 
                +2.600436,		 // Joint 6 
                +0.832778		 // Joint 7 
        };

        // Position near storage place 2
        positions["near storage place 2"] =
        {
                +1.853758,		 // Joint 1 
                +0.514412,		 // Joint 2 
                +2.624540,		 // Joint 3 
                -1.938238,		 // Joint 4 
                -0.033951,		 // Joint 5 
                +2.403973,		 // Joint 6 
                +0.527461		 // Joint 7 
        };

        // Position near storage place 2
        positions["storage place 2"] =
        {
                +1.841644,		 // Joint 1 
                +0.319584,		 // Joint 2 
                +2.607986,		 // Joint 3 
                -1.886534,		 // Joint 4 
                +0.033933,		 // Joint 5 
                +2.464868,		 // Joint 6 
                +0.527462		 // Joint 7 
        };

        // Position near storage place 3
        positions["near storage place 3"] =
        {
                +1.793602,		 // Joint 1 
                +0.293711,		 // Joint 2 
                +2.425398,		 // Joint 3 
                -1.710999,		 // Joint 4 
                +0.358704,		 // Joint 5 
                +2.326216,		 // Joint 6 
                +0.096573		 // Joint 7 
        };

        // Position near storage place 3
        positions["storage place 3"] =
        {
                +1.785067,		 // Joint 1 
                +0.111577,		 // Joint 2 
                +2.395085,		 // Joint 3 
                -1.661979,		 // Joint 4 
                +0.492932,		 // Joint 5 
                +2.331837,		 // Joint 6 
                +0.039065		 // Joint 7 
        };

        // Position near storage place 4
        positions["near storage place 4"] =
        {
                +1.704027,		 // Joint 1 
                +0.584230,		 // Joint 2 
                +2.875369,		 // Joint 3 
                -2.515814,		 // Joint 4 
                -0.455071,		 // Joint 5 
                +2.950280,		 // Joint 6 
                +1.117243		 // Joint 7 
        };

        // Position near storage place 4
        positions["storage place 4"] =
        {
                +1.725847,		 // Joint 1 
                +0.306364,		 // Joint 2 
                +2.863083,		 // Joint 3 
                -2.341458,		 // Joint 4 
                -0.448853,		 // Joint 5 
                +2.911509,		 // Joint 6 
                +1.117706		 // Joint 7 
        };

        // Position near storage place 5
        positions["near storage place 5"] =
        {
                +1.899778,		 // Joint 1 
                +0.622851,		 // Joint 2 
                +2.525775,		 // Joint 3 
                -2.343862,		 // Joint 4 
                -0.234449,		 // Joint 5 
                +2.633637,		 // Joint 6 
                +0.642411		 // Joint 7 
        };

        // Position near storage place 5
        positions["storage place 5"] =
        {
                +2.044198,		 // Joint 1 
                +0.296671,		 // Joint 2 
                +2.394162,		 // Joint 3 
                -2.164684,		 // Joint 4 
                +0.020413,		 // Joint 5 
                +2.641030,		 // Joint 6 
                +0.481424		 // Joint 7 
        };


        // Position near storage place 6
        positions["near storage place 6"] =
        {
                +2.439258,		 // Joint 1 
                +0.950777,		 // Joint 2 
                +1.958296,		 // Joint 3 
                -2.145058,		 // Joint 4 
                -0.788928,		 // Joint 5 
                +2.610972,		 // Joint 6 
                +0.723250		 // Joint 7 
        };

        // Position near storage place 6
        positions["storage place 6"] =
        {
                +2.660545,		 // Joint 1 
                +0.911755,		 // Joint 2 
                +1.782103,		 // Joint 3 
                -2.025104,		 // Joint 4 
                -0.883853,		 // Joint 5 
                +2.615542,		 // Joint 6 
                +0.767611		 // Joint 7 
        };

        // Position near storage place 7
        positions["near storage place 7"] =
        {
                +1.629001,		 // Joint 1 
                +0.332123,		 // Joint 2 
                +2.875330,		 // Joint 3 
                -2.677662,		 // Joint 4 
                -0.396351,		 // Joint 5 
                +3.259462,		 // Joint 6 
                +1.011538		 // Joint 7 
        };

        // Position near storage place 7
        positions["storage place 7"] =
        {
                +1.644874,		 // Joint 1 
                +0.044384,		 // Joint 2 
                +2.876314,		 // Joint 3 
                -2.513478,		 // Joint 4 
                -0.377289,		 // Joint 5 
                +3.348445,		 // Joint 6 
                +1.079716		 // Joint 7 
        };

        // Position near storage place 8
        positions["near storage place 8"] =
        {
                +1.405430,		 // Joint 1 
                +0.055462,		 // Joint 2 
                +2.618529,		 // Joint 3 
                -2.489355,		 // Joint 4 
                -1.104544,		 // Joint 5 
                +3.744352,		 // Joint 6 
                +1.480782		 // Joint 7 
        };

        // Position near storage place 8
        positions["storage place 8"] =
        {
                +1.432385,		 // Joint 1 
                -0.160770,		 // Joint 2 
                +2.681322,		 // Joint 3 
                -2.348030,		 // Joint 4 
                -0.984436,		 // Joint 5 
                +3.739133,		 // Joint 6 
                +1.481079		 // Joint 7 
        };


        // Position near storage place 9
        positions["near storage place 9"] =
        {
                +1.251620,		 // Joint 1 
                +0.139746,		 // Joint 2 
                +2.852862,		 // Joint 3 
                -2.300409,		 // Joint 4 
                +1.745586,		 // Joint 5 
                +2.658023,		 // Joint 6 
                -1.256127		 // Joint 7 
        };

        // Position near storage place 9
        positions["storage place 9"] =
        {
                +1.284026,		 // Joint 1 
                -0.101459,		 // Joint 2 
                +2.863415,		 // Joint 3 
                -2.128719,		 // Joint 4 
                +1.742942,		 // Joint 5 
                +2.609693,		 // Joint 6 
                -1.250185		 // Joint 7 
        };

        // Position above desk - pointing to camera
        positions["camera"] =
        {
                +0.861129,		 // Joint 1 
                +0.835717,		 // Joint 2 
                +0.199601,		 // Joint 3 
                -0.577428,		 // Joint 4 
                -0.169529,		 // Joint 5 
                +3.432604,		 // Joint 6 
                +0.994774		 // Joint 7 
        };

        // Position above desk - pointing to camera
        positions["desk right"] =
        {
                +0.428084,		// Joint 1
                +1.016777,		// Joint 2
                +0.352650,		// Joint 3
                -1.439887,		// Joint 4
                -0.625543,		// Joint 5
                +2.445297,		// Joint 6
                +1.911189		// Joint 7
        };

        // Position above desk - pointing to camera
        positions["near desk right"] =
        {
                +0.424486,		// Joint 1
                +0.827581,		// Joint 2
                +0.357930,		// Joint 3
                -1.493054,		// Joint 4
                -0.566994,		// Joint 5
                +2.321353,		// Joint 6
                +1.885754		// Joint 7
        };

        // Position above desk - pointing to camera
        positions["desk left"] =
        {
                -1.144081,		// Joint 1
                +1.164832,		// Joint 2
                +0.432686,		// Joint 3
                -1.217856,		// Joint 4
                -0.331640,		// Joint 5
                +2.463407,		// Joint 6
                +0.085543		// Joint 7
        };

        // Position above desk - pointing to camera
        positions["near desk left"] =
        {
                -0.986104,		// Joint 1
                +0.979894,		// Joint 2
                +0.253666,		// Joint 3
                -1.205803,		// Joint 4
                -0.258403,		// Joint 5
                +2.240249,		// Joint 6
                +0.130150		// Joint 7
        };



        // Positions for fat cup gripper just in front of the storage place
        // TOP position above cup storage
        positions["pack pose"] =
        {
                -0.000164,              // Joint 1
                -0.559858,              // Joint 2
                -0.000322,              // Joint 3
                -2.600814,              // Joint 4
                +0.000620,              // Joint 5
                +1.854628,              // Joint 6
                +0.785073               // Joint 7
        };

        // initial approach
        positions["cups init"] =
        {
                -1.566413,              // Joint 1
                -1.393163,              // Joint 2
                -0.069446,              // Joint 3
                -2.640760,              // Joint 4
                +0.014970,              // Joint 5
                +2.712432,              // Joint 6
                +0.781679               // Joint 7
        };

        // near cup 1
        positions["near cup 1"] =
        {
                -1.579210,              // Joint 1
                -0.830873,              // Joint 2
                -0.080680,              // Joint 3
                -2.355923,              // Joint 4
                +0.000934,              // Joint 5
                +2.715878,              // Joint 6
                +0.766539               // Joint 7
        };

        // cup 1
        positions["cup 1"] =
        {
                -1.552766,              // Joint 1
                -0.578903,              // Joint 2
                -0.085411,              // Joint 3
                -2.239540,              // Joint 4
                -0.081153,              // Joint 5
                +2.889223,              // Joint 6
                +0.802551               // Joint 7
        };

	// near cup 2
        positions["near cup 2"] =
        {
		-1.592111,		// Joint 1
		-0.831132,		// Joint 2
		-0.293202,		// Joint 3
		-2.334463,		// Joint 4
		+0.001557,		// Joint 5
		+2.714610,		// Joint 6
		+0.533747		// Joint 7
	};

        // cup 2
        positions["cup 2"] =
        {
                -1.542355,		// Joint 1
		-0.520639,		// Joint 2
		-0.290363,		// Joint 3
		-2.202321,		// Joint 4
		+0.070173,		// Joint 5
		+2.963892,		// Joint 6
		+0.559834		// Joint 7
        };

	// near cup 3
        positions["near cup 3"] =
        {
		-1.192797,		// Joint 1
		-0.846124,		// Joint 2
		-0.781272,		// Joint 3
		-2.085752,		// Joint 4
		+0.024559,		// Joint 5
		+2.608819,		// Joint 6
		+0.188706		// Joint 7
        };

        // cup 3
        positions["cup 3"] =
        {
                -0.933278,		// Joint 1
		-0.592505,		// Joint 2
		-0.968288,		// Joint 3
		-1.951560,		// Joint 4
		+0.025300,		// Joint 5
		+2.786079,		// Joint 6
		+0.235363		// Joint 7
        };

	// near cup 4
        positions["near cup 4"] =
        {
		-1.590214,		// Joint 1
		-0.938238,		// Joint 2
		-0.144173,		// Joint 3
		-2.652717,		// Joint 4
		+0.146305,		// Joint 5
		+2.848828,		// Joint 6
		+0.435233		// Joint 7
        };

        // cup 4
        positions["cup 4"] =
        {
                -1.543271,		// Joint 1
		-0.547294,		// Joint 2
		-0.167394,		// Joint 3
		-2.437655,		// Joint 4
		+0.143427,		// Joint 5
		+2.856439,		// Joint 6
		+0.468385		// Joint 7
        };

	// near cup 5
        positions["near cup 5"] =
        {
                -1.588526,		// Joint 1
		-0.936532,		// Joint 2
		-0.391557,		// Joint 3
		-2.677443,		// Joint 4
		+0.516925,		// Joint 5
		+2.855881,		// Joint 6
		-0.058971		// Joint 7
        };

        // cup 5
        positions["cup 5"] =
        {
                -1.442409,		// Joint 1
		-0.513542,		// Joint 2
		-0.410451,		// Joint 3
		-2.397952,		// Joint 4
		+0.472313,		// Joint 5
		+2.930697,		// Joint 6
		+0.092695		// Joint 7
        };

	// near cup 6
        positions["near cup 6"] =
        {
                -1.626045,		// Joint 1
		-0.678236,		// Joint 2
		-0.596773,		// Joint 3
		-2.411235,		// Joint 4
		+0.702596,		// Joint 5
		+2.675053,		// Joint 6
		-0.389695		// Joint 7
        };

        // cup 6
        positions["cup 6"] =
        {
                -1.338732,		// Joint 1
		-0.415994,		// Joint 2
		-0.687103,		// Joint 3
		-2.236185,		// Joint 4
		+0.852336,		// Joint 5
		+2.914067,		// Joint 6
		-0.327671		// Joint 7
        };

	// near cup 7
        positions["near cup 7"] =
        {
                -0.956256,		// Joint 1
		-1.461425,		// Joint 2
		-1.454781,		// Joint 3
		-2.783293,		// Joint 4
		-0.082922,		// Joint 5
		+2.802803,		// Joint 6
		-0.602754		// Joint 7
        };

        // cup 7
        positions["cup 7"] =
        {
		-0.517795,		// Joint 1
		-1.459124,		// Joint 2
		-1.491728,		// Joint 3
		-2.596884,		// Joint 4
		-0.088874,		// Joint 5
		+2.920346,		// Joint 6
		-0.600518		// Joint 7
        };

	// near cup 8
        positions["near cup 8"] =
        {
                -1.098623,		// Joint 1
		-1.113528,		// Joint 2
		-1.257720,		// Joint 3
		-2.548568,		// Joint 4
		+0.301772,		// Joint 5
		+2.551383,		// Joint 6
		-0.634836		// Joint 7
        };

        // cup 8
        positions["cup 8"] =
        {
                -0.631484,		// Joint 1
		-1.120709,		// Joint 2
		-1.390848,		// Joint 3
		-2.470007,		// Joint 4
		+0.303369,		// Joint 5
		+2.954078,		// Joint 6
		-0.604710		// Joint 7
        };

	// near cup 9
        positions["near cup 9"] =
        {
                -1.151302,		// Joint 1
		-0.935090,		// Joint 2
		-1.167966,		// Joint 3
		-2.546486,		// Joint 4
		+0.469172,		// Joint 5
		+2.873349,		// Joint 6
		-0.512533		// Joint 7
        };

        // cup 9
        positions["cup 9"] =
        {
                -0.681923,		// Joint 1
		-0.932973,		// Joint 2
		-1.429585,		// Joint 3
		-2.302971,		// Joint 4
		+0.319224,		// Joint 5
		+2.886175,		// Joint 6
		-0.503936		// Joint 7
        };

        // Position above cart
        positions["cart init"] =
        {
                +0.006132,              // Joint 1
                -1.272309,              // Joint 2
                -0.043135,              // Joint 3
                -2.619795,              // Joint 4
                +0.010689,              // Joint 5
                +2.711010,              // Joint 6
                +0.784954               // Joint 7
        };

        // Position above cart
        positions["near cart position"] =
        {
                -0.857337,		// Joint 1
                -1.465075,		// Joint 2
                +1.648837,		// Joint 3
                -2.449865,		// Joint 4
                +1.534230,		// Joint 5
                +2.910359,		// Joint 6
                +0.920835		// Joint 7
        };

        // Position at cart slot 1
        positions["final cart position 1"] =
        {
                -2.529543,		// Joint 1
                -1.650897,		// Joint 2
                +1.970968,		// Joint 3
                -0.732352,		// Joint 4
                -0.476340,		// Joint 5
                +2.980933,		// Joint 6
                +2.692670		// Joint 7
        };

        // Position at cart slot 2
        positions["final cart position 2"] =
        {
                -2.565061,		// Joint 1
                -1.621095,		// Joint 2
                +1.980658,		// Joint 3
                -0.890550,		// Joint 4
                -0.475252,		// Joint 5
                +2.983565,		// Joint 6
                +2.672337		// Joint 7
        };

        // Position at cart slot 3
        positions["final cart position 3"] =
        {
                -2.663470,		// Joint 1
                -1.617490,		// Joint 2
                +1.979150,		// Joint 3
                -0.931156,		// Joint 4
                -0.475181,		// Joint 5
                +2.983441,		// Joint 6
                +2.673147		// Joint 7
        };


        // Position at cart - end effector turned around
        positions["low cart position"] =
        {
                -2.618789,		// Joint 1
                -1.554653,		// Joint 2
                +2.508276,		// Joint 3
                -0.766913,		// Joint 4
                -2.313540,		// Joint 5
                +2.646845,		// Joint 6
                -2.385139		// Joint 7
        };

        return positions[position];
}

