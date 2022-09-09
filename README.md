# marker_vizualiser


# Installation

Pour installer ce noeud il faut le cloner dans le dossier src de votre catkin workspace (catkin_ws)

commande a suivre:


 	cd catkin_ws/src
 	
 	git clone https://github.com/vanexcel777/marker_visualizer.git
 	
 	catkin build
 	
	cd ..
	
	source devel/setup.bash
	

# compilation

 
 ### 1er terminal 
 
 lancement du server
 
 
 
	Ouvrez un premier terminal: ctrl + alt + t  

	lancer le roscore: roscore






 ### 2e terminal 

Afin de resumer l execution sur le 'rosrun', le roslauch peut facilite  la tache 

  	Ouvrez un autre  terminal: ctrl + alt + t  
  
 	aller dans le dossier catkin_ws: cd catkin_ws
  
	sourcer:  source devel/setup.bash

	executer :  roslaunch marker_vizualiser marker_vizualiser.launch

 
 
 ### 3e terminal 
 
 
  lancement du rviz
 

 
	Ouvrez un autre  terminal: ctrl + alt + t  
 
	aller dans le dossier catkin_ws: cd catkin_ws
 
	sourcer: source le devel/setup.bash
 
	executer :  rosrun rviz rviz 
  
	clicker sur : "add" -> "by topic" ->  'pose'(sur /marker_vizualiser) -> ok
  
  


 
 # astuce
 
 appuyer deux fois sur l interface graphique pour regarder l evolution sur rviz
