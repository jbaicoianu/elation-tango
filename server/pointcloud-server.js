var net = require('net');
var ws = require('ws');

var serverPort = 9917;
var websocketPort = 9918;

var appClients = {};
var websocketClients = [];

console.log('Create app server on TCP port ' + serverPort);
net.createServer(function(socket) {
  var clientid = socket.remoteAddress + ':' + socket.remotePort;
  appClients[clientid] = socket;

  console.log('new client: ' + clientid);
  
  var recvbuf = new Buffer(0);

  socket.on('data', function(data) {
    var start = recvbuf.length;
    recvbuf = Buffer.concat([recvbuf, data]);

    var headersize = 8; // [ uint8 | uint8 | uint8 | uint8 | uint32 ]

    var type = recvbuf.readUInt8(0);
    var size = recvbuf.readUInt32LE(1);

    //console.log('new data: ', type, size, data.length);

    if (recvbuf.length >= size + headersize) {
      var payload = recvbuf.slice(headersize, size + headersize);
      recvbuf = recvbuf.slice(size + headersize);
      var msg = createAppMessage(type, payload);
      console.log('NEW MESSAGE:', msg.summary());
      var msgJSON = msg.serializeJSON();
      for (var i = 0; i < websocketClients.length; i++) {
        try {
          websocketClients[i].send(msgJSON);
        } catch (e) {
          console.log("ERROR couldn't send to websocket " + i + ", socket closed?");
        }
      }
    } else {
      //console.log('not enough to parse yet, keep going');
    }
    
  });
  socket.on('end', function() {
    console.log('client disconnected: ' + clientid);
    delete appClients[clientid];
  });
}).listen(serverPort);

console.log('Create websocket server on port ' + websocketPort);
var wss = new ws.Server({port: websocketPort});
wss.on('connection', function(ws) {
  console.log('new websocket client');
  websocketClients.push(ws);
  ws.on('message', function(msg) {
    console.log('new message: ', msg);
  });
  ws.on('close', function() {
    var idx = websocketClients.indexOf(ws);
    if (idx != -1) {
      console.log('removed websocket connection ' + idx);
      websocketClients.splice(idx, 1);
    }
  });
});

function AppMessage(data) {
  this.type = "unknown";
  this.parsePayload(data);
}
AppMessage.prototype.parsePayload = function(data) {
}
AppMessage.prototype.summary = function() {
  return "AppMessage: " + this.type;
}
AppMessage.prototype.serializeJSON = function() {
  return JSON.stringify(this);
}

function AppMessageTangoIntro(data) {
  this.type = "tango_intro";
  this.id = null;
  this.fov = null;
  this.aspect = null;
  this.far = null;
  this.parsePayload(data);
}
AppMessageTangoIntro.prototype = Object.create(AppMessage.prototype);
AppMessageTangoIntro.prototype.parsePayload = function(data) {
  this.fov = data.readFloatLE(0);
  this.aspect = data.readFloatLE(4);
  this.far = data.readDoubleLE(8);
  this.id = data.toString('utf8', 16);
}
AppMessageTangoIntro.prototype.summary = function() {
  return "AppMessageTangoIntro: " + this.id;
}

function AppMessageTangoPose(data) {
  this.type = "tango_pose";
  this.parsePayload(data);
}
AppMessageTangoPose.prototype = Object.create(AppMessage.prototype);
AppMessageTangoPose.prototype.parsePayload = function(data) {
  // Message format:
  // ---------------------------
  // | type        | uint8     |
  // | translation | double[3] |
  // | orientation | double[4] |
  // ---------------------------
  var size = 8;
  this.translation = [
    data.readDoubleLE(size*0),
    data.readDoubleLE(size*1),
    data.readDoubleLE(size*2)
  ];

  this.orientation = [
    data.readDoubleLE(size*3),
    data.readDoubleLE(size*4),
    data.readDoubleLE(size*5),
    data.readDoubleLE(size*6)
  ];

}
AppMessageTangoPose.prototype.summary = function() {
  return "AppMessageTangoPose: translation(" + this.translation[0].toFixed(3) + ", " + this.translation[1].toFixed(3) + ", " + this.translation[2].toFixed(3) + ") orientation(" + this.orientation[0].toFixed(3) + ", " + this.orientation[1].toFixed(3) + ", " + this.orientation[2].toFixed(3) + ", " + this.orientation[3].toFixed(3) + ")";
}

function AppMessageTangoPoints(data) {
  this.type = "tango_points";
  this.parsePayload(data);
}
AppMessageTangoPoints.prototype = Object.create(AppMessage.prototype);
AppMessageTangoPoints.prototype.parsePayload = function(data) {
  // Message format:
  // -------------------------------------
  // | type          | uint8             |
  // | numpoints     | int32             |
  // | pointdata     | float[numpoints*3]|
  // | colordata     | uint8[numpoints*3]|
  // -------------------------------------

  this.numpoints = data.readUInt32LE(0);
  var pointoffset = 4;

  //var pointdata = new Float32Array(data, pointoffset, this.numpoints * 3);

  this.pointdata = [];
  this.colordata = [];
  for (var i = 0; i < this.numpoints; i++) {
    var offset = pointoffset + (i * 3 * 4),
        coloroffset = pointoffset + (3 * this.numpoints * 4) + (i * 3);
    this.pointdata[i] = [data.readFloatLE(offset), data.readFloatLE(offset + 4), data.readFloatLE(offset + 8), data.readUInt32LE(pointoffset + (this.numpoints * 4) + (i * 4))];
    this.colordata[i] = [data.readUInt8(coloroffset), data.readUInt8(coloroffset + 1), data.readUInt8(coloroffset + 2)];
  }
}
AppMessageTangoPoints.prototype.summary = function() {
  return "AppMessageTangoPoints: " + this.numpoints + " points";
}

var AppMessageTypes = {
  1: AppMessageTangoIntro,
  2: AppMessageTangoPose,
  3: AppMessageTangoPoints
}
function createAppMessage(type, payload) {
  if (AppMessageTypes[type]) {
    return new AppMessageTypes[type](payload);
  }
  return new AppMessage(payload);
}


