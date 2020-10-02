# turtlesim_snake
Instrucciones:

  Ejercicio 2:
    Para que la tortuga 2 generada siga a la 1 cuando ésta se acerca, he llevado a cabo un método que permite determinar la distancia a la que está una tortuga de    la otra y cuando esa distancia es menor de un valor establecido, se lo proporciona una velocidad lineal y angular a la tortuga 2 para que siga a la 1.
    
    roslaunch turtlesim_snake start_exercise2.launch
    
   Ejercicio 3:
      En este ejercicio he modificado la función del ejercicio anterior para que sea un servicio (server) de tipo std_srvs/Empty.
      
      roslaunch turtlesim_snake start_exercise3.launch
      
   Ejercicio 4:
      Para este ejercicio he creado un servicio de tipo Turtle_pose.srv, creado por mi, cuyos párametros de entrada serán x, y, theta. Esto permite darle unos argumentos de entrada que proporcionen la posición y la orientación de la tortuga como se pide.
      
      roslaunch turtlesim_snake start_exercise4.launch
      
   Ejercicio 5/6:
      Para estos dos ejercicios he creado dos nodos turtle_spawn y snake_game. El primero permitirá generar tortugas de forma aleatoria cada cierto tiempo y llevar un registro del número de tortugas que se generan. El segundo nodo ejecutará el juego, en el que a medida que la tortuga se mueve y se acerca a las tortugas generadas éstas se añadirán detrás formando una fila sin llegar a colisionar las unas con las otras. 
      Se han implementado también ciertas funciones que permiten que el color del fondo cambie al alcanzar otra tortuga, el juego tendrá niveles en los que se deberán captar ciertas tortugas para subir de nivel, teniendo en cuenta que si se colisiona con la propia fila de tortugas se perderá y el juego se reiniciará. 
      
      roslaunch turtlesim_snake start_exercise5.launch
