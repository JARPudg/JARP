# JARP
1. test.py es usado para probar YOLOv8 en el Jetson Nano
1. arm.py este codigo es usado para comprobar la conexion serial entre el Jetson Nano y Pixhawk, nosotros tubimos un problema al probar el codigo ya que el puerto serial no dejaba leer ni escribir asi que usamos el siguiente codigo: "sudo chmod 777 /dev/ttyTHS1" este codigo en terminal se usa para poner el puerto ttyTHS1 este por completo abierto a lectura y escritura, para comprobar que el puerto esta funcionando podemos usar este otro codigo: "ls -l /dev/ttyTHS1" el resultado de este codigo en terminal debe de ser el siguiente, crwxrwxrwx 1 root tty "numero", "fecha hora" /dev/ttyTHS1.
1. main.py es el codigo final usado para mover el drone solo en Yaw usando un PID 
