## Objective

Specify, design, implement and debug a multi-task (multi-threaded) embedded project using a real-time operating system.


#### Functional Specification: MP3 player with TouchScreen

1. A Touch-screen MP3 player. The GUI shows a stop, play, next and prev buttons
2. When a button pressed occurred from user, the mp3 tasks either play, stop, plays next music or previous music
3. Used mutex to control SPI access for GUI tasks for printing UI on touch screen and accessing sdcard when reading data
4. Used queue for inter task synchronization/communication
5. UI will also print on screen what song is being played

## BLOCK DIAGRAM
![Image](https://github.com/user-attachments/assets/5aaa155f-a459-4c8d-a748-5fc81f9ff7eb)


## ARCHITECTURAL DESIGN
![Image](https://github.com/user-attachments/assets/f4ce0400-c710-4a18-98cf-9bc4c1edc62c)