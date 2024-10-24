const net = require("net");

// Create a TCP server
const server = net.createServer((socket) => {
  console.log("Client connected");

  socket.write("Connected");

  // Handle incoming data from the client
  socket.on("data", (data) => {
    console.log("Received:", data.toString());

    const response = "Hello from TCP server!\n";

    socket.write(response);
  });

  // Handle client disconnection
  socket.on("end", () => {
    console.log("Client disconnected");
  });

  // Handle errors
  socket.on("error", (err) => {
    console.error("Socket error:", err);
  });
});

// The server listens on port 4242
const PORT = 4242;
server.listen(PORT, () => {
  console.log(`Server listening on port ${PORT}`);
});

// Handle server errors
server.on("error", (err) => {
  console.error("Server error:", err);
});
