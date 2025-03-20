function Decoder(bytes, port, uplink_info) {

  // Ensure the payload length matches the expected size
  if (bytes.length !== 16) {
    throw new Error("Invalid payload size. Expected 16 bytes.");
  }

  // Helper function to read a 16-bit signed integer
  function readInt16LE(offset) {
    return (bytes[offset] | (bytes[offset + 1] << 8)) << 16 >> 16; // Convert to signed 16-bit
  }

  // Helper function to read a 16-bit unsigned integer
  function readUInt16LE(offset) {
    return bytes[offset] | (bytes[offset + 1] << 8); // Convert to unsigned 16-bit
  }

  // Decode each value from the payload
  let offset = 0;
  const dirAvg = readInt16LE(offset) / 10.0; // Scale back to 1 decimal place
  offset += 2;
  const velAvg = readInt16LE(offset) / 10.0; // Scale back to 1 decimal place
  offset += 2;
  const gust = readInt16LE(offset) / 10.0; // Scale back to 1 decimal place
  offset += 2;
  const lull = readInt16LE(offset) / 10.0; // Scale back to 1 decimal place
  offset += 2;
  const batVoltageF = readInt16LE(offset) / 100.0; // Scale back to 2 decimal places
  offset += 2;
  const capVoltageF = readInt16LE(offset) / 100.0; // Scale back to 2 decimal places
  offset += 2;
  const temperatureF = readInt16LE(offset) / 10.0; // Scale back to 1 decimal place
  offset += 2;
  const rain = readUInt16LE(offset) / 10.0; // Scale back to 1 decimal place
  offset += 2;

  // Return the decoded values as a JSON object
  return JSON.stringify({
    dir: dirAvg,
    velAvg: velAvg,
    gust: gust,
    lull: lull,
    batV: batVoltageF,
    capV: capVoltageF,
    tempC: temperatureF,
    rainmm: rain
  });
}