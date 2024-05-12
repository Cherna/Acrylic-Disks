// void runSequentially(std::array<AccelStepper*, 3> circle = circle1) {
//   static int counter = 0;
//   static unsigned long previousMillis = 0;
//   static int direction = 1;
//   if (millis() - previousMillis >= 500) {
//     previousMillis = millis();
//     Serial.print("Moving: ");
//     Serial.println(counter);
//   }
//   circle[counter]->move(200 * MICSTEP * direction);

//   // Increase motor counter
//   counter = counter == getArrayLength(circle) - 1 ? 0 : counter + 1;

//   // Reverse the direction after all motors have run
//   if (counter >= getArrayLength(circle) - 1) {
//     Serial.println("Resetting counter and reversing");
//     counter = 0;
//     direction *= -1;
//   }
// }