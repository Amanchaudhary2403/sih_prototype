const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const { spawn } = require('child_process');
const path = require('path');
const cors = require('cors');

const app = express();
app.use(cors());
app.use(express.static(path.join(__dirname, 'public')));

const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: '*',
    methods: ['GET', 'POST']
  }
});

let pythonProcess = null;

function startPythonEngine() {
  if (pythonProcess) {
    pythonProcess.kill();
  }
  
  pythonProcess = spawn('python3', ['-u', '../sim_engine.py'], {
    cwd: __dirname
  });

  pythonProcess.stdout.on('data', (data) => {
    const lines = data.toString().split('\n');
    for (const line of lines) {
      if (line.trim()) {
        try {
          const frame = JSON.parse(line);
          io.emit('frame', frame);
        } catch (e) {
          // Ignore non-JSON lines (e.g. planner debug prints)
        }
      }
    }
  });

  pythonProcess.stderr.on('data', (data) => {
    console.error(`Python Stderr: ${data}`);
  });

  pythonProcess.on('close', (code) => {
    console.log(`Python process exited with code ${code}`);
  });
}

io.on('connection', (socket) => {
  console.log('Client connected:', socket.id);

  socket.on('command', (cmd) => {
    if (pythonProcess && pythonProcess.stdin) {
      pythonProcess.stdin.write(JSON.stringify(cmd) + '\n');
    }
  });

  socket.on('disconnect', () => {
    console.log('Client disconnected:', socket.id);
  });
});

startPythonEngine();

const PORT = process.env.PORT || 3001;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`Server listening on port ${PORT}`);
});
