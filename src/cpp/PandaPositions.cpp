
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
                -8.18497303143e-05,  // Joint 1
                -0.784857239927,  // Joint 2
                0.000271869333208,  // Joint 3
                -2.35662068299,  // Joint 4
                0.000554219921981,  // Joint 5
                1.57110703584,  // Joint 6
                0.78476321034   // Joint 7
        };



        // initial approach
        positions["cups init"] =
        {
                -2.10463079074,  // Joint 1
                -1.08965928206,  // Joint 2
                -0.128359101328,  // Joint 3
                -2.50268168282,  // Joint 4
                1.10388845417,  // Joint 5
                2.59828735727,  // Joint 6
                2.72814171152   // Joint 7
        };


        // near cup 1
        positions["near cup 1"] =
        {
                -1.91612346345,  // Joint 1
                -0.829865015061,  // Joint 2
                0.238380504703,  // Joint 3
                -2.35844629841,  // Joint 4
                1.53393006312,  // Joint 5
                2.99396240292,  // Joint 6
                2.58604482318   // Joint 7
        };



        // cup 1
        positions["cup 1"] =
        {
                -1.91531575028,  // Joint 1
                -0.577898670039,  // Joint 2
                0.277566605479,  // Joint 3
                -2.18619800195,  // Joint 4
                1.45007618878,  // Joint 5
                2.99202729283,  // Joint 6
                2.60025904679   // Joint 7
        };





	// near cup 2
        positions["near cup 2"] =
        {
                -2.04848527985,  // Joint 1
                -0.621048902499,  // Joint 2
                -0.0111836897831,  // Joint 3
                -2.18438044679,  // Joint 4
                1.1020965478,  // Joint 5
                2.5986323369,  // Joint 6
                2.77195599078   // Joint 7
        };



        // cup 2
        positions["cup 2"] =
        {
                -2.01990358857,  // Joint 1
                -0.452640077886,  // Joint 2
                0.033452693418,  // Joint 3
                -2.05362731093,  // Joint 4
                1.10215391768,  // Joint 5
                2.63632519045,  // Joint 6
                2.77260938912   // Joint 7
        };



	// near cup 3
        positions["near cup 3"] =
        {
                -2.00792702567,  // Joint 1
                -0.499379177131,  // Joint 2
                -0.250533672103,  // Joint 3
                -2.13490684473,  // Joint 4
                1.57394005113,  // Joint 5
                2.5043633314,  // Joint 6
                2.19999056071   // Joint 7
        };



        // cup 3
        positions["cup 3"] =
        {
                -1.96156912286,  // Joint 1
                -0.277884404547,  // Joint 2
                -0.229759243363,  // Joint 3
                -1.91700087736,  // Joint 4
                1.57394844952,  // Joint 5
                2.49914133165,  // Joint 6
                2.23721284603   // Joint 7
        };



	// near cup 4 {{{{{{{{{{{{{{ need to implement fro here }}}}}}}}}}}}}}
        positions["near cup 4"] =
        {
                -1.72456378939,  // Joint 1
                -0.844180738777,  // Joint 2
                0.0711595301021,  // Joint 3
                -2.6124897983,  // Joint 4
                2.57400804689,  // Joint 5
                2.9590192627,  // Joint 6
                1.44061687135   // Joint 7
        };





        // cup 4
        positions["cup 4"] =
        {
                -1.79323978399,  // Joint 1
                -0.510895366167,  // Joint 2
                0.0688173098479,  // Joint 3
                -2.40202056402,  // Joint 4
                2.57391188917,  // Joint 5
                2.7914799982,  // Joint 6
                1.43633014904   // Joint 7
        };





	// near cup 5
        positions["near cup 5"] =
        {
                -2.0206315746,  // Joint 1
                -0.734823920442,  // Joint 2
                -0.0444405451331,  // Joint 3
                -2.64280007924,  // Joint 4
                2.10031899267,  // Joint 5
                2.55755504342,  // Joint 6
                1.68886415306   // Joint 7
        };


        // cup 5
        positions["cup 5"] =
        {
                -1.98609189972,  // Joint 1
                -0.420916557393,  // Joint 2
                0.0193834120458,  // Joint 3
                -2.34611922916,  // Joint 4
                2.13195943647,  // Joint 5
                2.62676806863,  // Joint 6
                1.70819802735   // Joint 7
        };


	// near cup 6
        positions["near cup 6"] =
        {
                -2.04268336824,  // Joint 1
                -0.532769191615,  // Joint 2
                -0.324283131625,  // Joint 3
                -2.45536747371,  // Joint 4
                1.7788146223,  // Joint 5
                2.40554012022,  // Joint 6
                1.79676623844   // Joint 7
        };


        // cup 6
        positions["cup 6"] =
        {
                -1.93344430897,  // Joint 1
                -0.217992827188,  // Joint 2
                -0.323770683633,  // Joint 3
                -2.16024427683,  // Joint 4
                1.81431411717,  // Joint 5
                2.36176282458,  // Joint 6
                1.80901546735   // Joint 7
        };


	// near cup 7
        positions["near cup 7"] =
        {
                -1.92980766817,  // Joint 1
                -0.616604185113,  // Joint 2
                -0.224109153235,  // Joint 3
                -2.89870575907,  // Joint 4
                2.01090178622,  // Joint 5
                2.15101931947,  // Joint 6
                1.39172183146   // Joint 7
        };


        // cup 7
        positions["cup 7"] =
        {
                -1.83163550682,  // Joint 1
                -0.306904663038,  // Joint 2
                -0.00899353923531,  // Joint 3
                -2.63295000701,  // Joint 4
                2.45880254244,  // Joint 5
                2.26035123796,  // Joint 6
                1.2394900109   // Joint 7
        };



	// near cup 8
        positions["near cup 8"] =
        {
                -2.05919401222,  // Joint 1
                -0.586351445641,  // Joint 2
                -0.230679986228,  // Joint 3
                -2.87942610151,  // Joint 4
                2.20696967576,  // Joint 5
                2.22120105291,  // Joint 6
                1.32205101687   // Joint 7
        };


        // cup 8
        positions["cup 8"] =
        {
                -1.91868864049,  // Joint 1
                -0.24431818053,  // Joint 2
                -0.134475936871,  // Joint 3
                -2.60053977275,  // Joint 4
                2.51449872931,  // Joint 5
                2.21172835897,  // Joint 6
                1.11467519587   // Joint 7
        };


	// near cup 9
        positions["near cup 9"] =
        {
                -2.12989212618,  // Joint 1
                -0.315124112542,  // Joint 2
                -0.390308081519,  // Joint 3
                -2.6037152449,  // Joint 4
                1.94523100666,  // Joint 5
                2.12424094467,  // Joint 6
                1.41917996051   // Joint 7
        };


        // cup 9
        positions["cup 9"] =
        {
                -2.01488530487,  // Joint 1
                -0.00452550545346,  // Joint 2
                -0.328690348709,  // Joint 3
                -2.30402307034,  // Joint 4
                2.04269225217,  // Joint 5
                2.08763738966,  // Joint 6
                1.44880202791   // Joint 7
        };



        // Position above cart
        positions["cart init"] =
        {
		-1.2671365723,  // Joint 1
		-0.759951032139,  // Joint 2
		1.12266015855,  // Joint 3
		-2.01386364565,  // Joint 4
		2.46298082051,  // Joint 5
		2.77987015429,  // Joint 6
		2.16948271089   // Joint 7
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

