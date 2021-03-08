#!/bin/sh
webotsWorld=1
for (( ; ; )) #outer loop
do
	clear
	echo "====================== Enter an option from the following: ======================"
	echo "	0: QUIT."
	echo "	1: IMACS with WEBOTS."
	echo "	2: IMACS with VREP."
	echo "	3: GENERATE doxygen documentation."
	echo -n "Option: "
	read IMACS_OPTION
	if [[ $IMACS_OPTION -eq 0 ]]
	then
		exit 0	
	elif [ $IMACS_OPTION -eq 3 ]
	then
		echo "make clean_doc"
		make clean_doc
		echo "make doc"
		make doc
		read -p "Enter to continue."
	elif [[ $IMACS_OPTION -eq 1 ]] || [[ $IMACS_OPTION -eq 2 ]]
	then	
		compiledFlag=0
		livePlot=0
		skipOptions=0
		webotsWorldEntered=0		
		for (( ; ; )) #inner loop
		do
			if [ $compiledFlag -eq 0 ]
			then
				echo "============================ STARTING COMPILATION ==============================="
				if [ $IMACS_OPTION -eq 1 ]
				then	
					echo "qmake imacs_webots_framework.pro"
					qmake imacs_webots_framework.pro
				else
					echo "qmake imacs_vrep_framework.pro"
					qmake imacs_vrep_framework.pro
				fi
				echo "make"
				make
				echo ""
				echo "============================ COMPILATION COMPLETED =============================="
				echo ""
				echo "If there is a compilation error, quit this script, rectify the error and then rerun this bash script."
				echo ""
				compiledFlag=1
			fi #end [$compiledFlag -eq 1]
			if [ $skipOptions -eq 0 ]
			then
				echo "====================== Enter an option from the following: ======================"
				echo "	0: QUIT."
				if [ $IMACS_OPTION -eq 1 ]
				then	
					echo "	1: SIMULATE IMACS WEBOTS. Choose this option ONLY if a webots scene is already open."
					echo "	   If a simulation was running in WEBOTS, Reset Simulation (Ctrl+Shift+T) inside the open webots scene first."
					echo "	2: OPEN the webots-scene (CityStraight). Choose this option if no webots scene is open."
					echo "	3: OPEN the webots-scene (CityStraightNight). Choose this option if no webots scene is open."
					echo "	4: KILLALL webots scenes. The terminal needs to be manually closed though."
				else
					echo "	1: SIMULATE IMACS VREP. Choose this option ONLY if a vrep scene is already open."
					echo "	   If a simulation was running in VREP,stop simulation first."
					echo "	2: OPEN the vrep-scene (StraightRoad). Choose this option if no vrep scene is open."
					echo "	3: OPEN the vrep-scene (CurveRoad). Choose this option if no vrep scene is open."
					echo "	4: KILLALL vrep scenes. The terminal needs to be manually closed though."
				fi
				echo "	5: OPEN live_plot for visualisation."
				echo "	6: MAKE CLEAN and recompile."
				echo "	7: (RE)COMPILE"
				echo "	8: QUIT to IMACS WEBOTS/VREP option menu. To switch to other simulator."
				echo -n "Option: "
				read OPTION
			fi #end [$skipOptions -eq 0]
			if [ $OPTION -eq 0 ]
			then
				exit 0
			elif [ $OPTION -eq 1 ]
			then
				clear
				echo -n "Enter scenario: "
				read SCENARIO
				if [ $livePlot -eq 0 ]
				then
					echo "gnome-terminal -- sh -c 'python live_plot.py; exec bash'"
					gnome-terminal -- sh -c 'python live_plot.py; exec bash'
					livePlot=1
				fi
				echo "============================ STARTING SIMULATION ==============================="
				if [ $IMACS_OPTION -eq 1 ]
				then
					if [ $webotsWorldEntered -eq 0 ]
					then
						echo -n "Enter the open/opened webots world (1: city_straight.wbt, 2: city_straight_night.wbt): "
						read webotsWorld
						webotsWorldEntered=1
					fi
					echo "./imacs_webots $SCENARIO $webotsWorld"
					./imacs_webots $SCENARIO $webotsWorld
				else
					echo "./imacs_vrep $SCENARIO"
					./imacs_vrep $SCENARIO
				fi
				echo ""
				echo "============================ COMPLETED SIMULATION =============================="
				read -p "Enter to continue."
				skipOptions=0
			elif [ $OPTION -eq 2 ] 
			then
				if [ $IMACS_OPTION -eq 1 ]
				then
					gnome-terminal -- sh -c 'cd webots_scenes; webots city_straight.wbt; exec bash'
				else
					gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/StraightRoad.ttt; exec bash'
				fi
				OPTION=1
				skipOptions=1
				webotsWorld=1
				webotsWorldEntered=1
				read -p "Enter after the scene is OPENED."
			elif [ $OPTION -eq 3 ]
			then
				if [ $IMACS_OPTION -eq 1 ]
				then
					gnome-terminal -- sh -c 'cd webots_scenes; webots city_straight_night.wbt; exec bash'
				else
					gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/CurveRoad.ttt; exec bash'
				fi
				OPTION=1
				skipOptions=1
				webotsWorld=2
				webotsWorldEntered=1
				read -p "Enter after the scene is OPENED."
			elif [ $OPTION -eq 4 ]
			then
				if [ $IMACS_OPTION -eq 1 ]
				then
					killall -9 webots-bin
				else
					killall -9 coppeliaSim
				fi
			elif [ $OPTION -eq 5 ]
			then
				echo "gnome-terminal -- sh -c 'python live_plot.py; exec bash'"				
				gnome-terminal -- sh -c 'python live_plot.py; exec bash'
				livePlot=1
			elif [ $OPTION -eq 6 ]
			then
				make clean
				compiledFlag=0		
			elif [ $OPTION -eq 7 ]
			then		
				compiledFlag=0	
			elif [ $OPTION -eq 8 ]
			then
				break		
			else #$OPTION > 8
				clear
			fi #end $OPTION
			clear
		done # end inner loop
	else #$IMACS_OPTION != {1,2,3}
		clear
	fi #end [ $IMACS_OPTION ]
done #outer loop
