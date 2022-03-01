# Práctica de navegación

## Eunciado:

[![GitHub Action
Status](https://github.com/Docencia-fmrico/navigation/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/navigation)


**Entrega:** Miércoles 2/3 

En la moqueta verde del laboratorio se limitará con unas paredes, y se pondrán obstáculos (cajas) dentro el viernes 25/2. No habrá cambios en el escenario desde este momento. El miércoles, al inicio de la clase se proporcionarán un conjunto de waypoints en un fichero de parámetros como este:

```
patrolling_node:
  ros__parameters:
    waypoints: ["wp1", "wp2"]
    wp1: [1.0, 1.0]
    wp2: [-1.0, -1,0]
```

El robot debe ir en orden la coordenada (x, y) de cada uno de ellos, emitiendo un sonido cuando lo considera alcanzado. Se cronometrará el tiempo que tarda en hacerlo.

La velocidad lineal no podrá ser nunca superior a 0.4. Se descalificará a quien incumpla esta regla.

Habrá dos rondas:

- Ronda 1: Habrá 4 waypoints, y ninguno en la posición de un obstáculo.
- Ronda 2: Habrá 3-7 waypoints, alguno de ellos en la posición de un obstáculo. En este caso, se podrá ir al siguiente en cuanto se detecte este caso.

## Solución

La resolución del ejercucio se ha llevado a cabo partiendo del código encontrado en el repositorio [https://github.com/Docencia-fmrico/navigation](https://github.com/Docencia-fmrico/navigation). En este enlace podemos encontrar la estructura del proyecto pero con algunas funcionalidades incompletas.

### Estructura del proyecto:
Dentro del respositorio podemos encontrar dos paquetes, uno dedicado al comportamiento del robot y otro dedicado a la navegación con el kobuki. Dentro del paquete "bt_behavior" nos encontramos con lo siguiente:

Organizaremos el código de manera que haya un programa principal que analiza los parámetros de entrada y llama a un Behavior Tree que será el que gestione el flujo de ejecución de código. A continuación se puede ver la estructura del Behavior Tree:

Podemos ver que únicamente tendremos tres acciones descritas a continuación:
- GetWayipoint: será la acción que usando los parámetros obtenidos en el fichero "waypoints.yaml" proporcionará el destino a la acción Move
- Move: esta acctión llamará a la navegación para que el robot alcance el destino propuesto anteriormente.
- Patrol: este Nodo se encargará de "desorientar" al robot.
Debido al bloque KeepRunningUntilFailure estas tres acciones estarán ejecutando constantemente hasta que cualquiera de ellas devuelva error, que únicamente puede pasar cuando la navegación falle.

Al obtener todos los waypoints del fichero de parámetros, los almacenaremos en un [double ended queue](https://en.cppreference.com/w/cpp/container/deque), nos resultará más facil para almacenar los datos y automáticamente liberarlos una vez usados. De ahí los iremos sacando usando el método pop_front que nos devolverá el primer elemento de la cola.

El segundo paquete llamado "kobuki_navigation" estará organizado de la siguiente manera:
- Parámetros de configuración: Nos encontramos con una carpeta que se llama params que incluye varios ficheros que contienen diversos parámetros que establecerán las especificaciones del robot como las velocidades angulares y lineales.
- Mapas: Dentro de esta carpeta podemos encontrar los ficheros que representan los mapas que vamos a usar. Uno de ellos será el que hemos creado en el laboratorio. Por cada mapa podemos encontrar un fichero .pgm que incluirá una imagen del mapa y otro fichero .yaml que establecerá los parámetros necesarios para iniciar el mapa en el visualizador como podrían ser las coordenadas de inicio.
- Launchers: Por último podemos ver que hay una carpeta launch que incluye dos ejecutables que iniciarán la navegación. El que usaremos es el llamado "kobuki_navigation.launch".

### Behavior Trees

### Navegación:

Recurriremos a la navegación proporcionada por [nav2](https://navigation.ros.org/index.html).

### Videos

## Contributors
* Rubén Montilla Fernández
* Blanca Soria Rubio
* Victor de la Torre Rosa
* Cristian Sánchez Rodríguez 
