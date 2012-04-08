"use strict";

var ROS = {
  url: 'ws://' + location.hostname + ':9090',
  socket: null,
  services: {},
  subscribers: {},

  call: function(topic, data, callback) {
    (this.services[topic] || (this.services[topic] = [])).push(callback);
    this.send({ receiver: topic, msg: data });
  },

  publish: function(topic, data, type) {
    this.send({ receiver: topic, msg: data, type: type });
  },

  subscribe: function(topic, callback) {
    (this.subscribers[topic] || (this.subscribers[topic] = [])).push(callback);
  },

  disconnect: function() {
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
    this.services = {};
    this.subscribers = {};
  },

  connect: function() {
    var self = this;
    this.disconnect();
    this.socket = new WebSocket(this.url);
    this.socket.onopen = function() {
      if (self.onopen) self.onopen();
    };
    this.socket.onclose = function() {
      self.disconnect();
      if (self.onclose) self.onclose();
      setTimeout(function() { self.connect(); }, 500);
    };
    this.socket.onerror = function() {
      self.disconnect();
      if (self.onclose) self.onclose();
      setTimeout(function() { self.connect(); }, 500);
    };
    this.socket.onmessage = function(e) {
      var data = JSON.parse(e.data);
      if (data.receiver in self.services) {
        var services = self.services[data.receiver];
        if (services.length) services.shift()(data.msg);
      } else if (data.receiver in self.subscribers) {
        self.subscribers[data.receiver].map(function(subscriber) {
          subscriber(data.msg);
        });
      } else if (self.onmessage) {
        self.onmessage(data);
      }
    };
    this.send = function(json) {
      var data = JSON.stringify(json);
      if (self.socket) self.socket.send(data);
    };
  }
};
