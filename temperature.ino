

void tem_update(void){ 
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
//  sensors.requestTemperatures(); 
  
  bluetooth.print("Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  bluetooth.print(19 + random(100)/100 - 0.5);//sensors.getTempCByIndex(0)); 
//  bluetooth.print(" - Fahrenheit temperature: ");
//  bluetooth.println(sensors.getTempFByIndex(0));
}
