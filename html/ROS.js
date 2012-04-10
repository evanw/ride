"use strict";

var ROS = {
  url: 'ws://' + location.hostname + ':9090',
  socket: null,
  nextID: 0,
  services: {},
  subscribers: {},

  call: function(topic, data, callback) {
    data.id = this.nextID++;
    var service = (this.services[topic] || (this.services[topic] = {}))
    service[data.id] = callback || null;
    this.send({ receiver: topic, msg: data });
  },

  publish: function(topic, data, type) {
    this.send({ receiver: topic, msg: data, type: type });
  },

  subscribe: function(topic, callback) {
    var subscriber = (this.subscribers[topic] || (this.subscribers[topic] = []));
    subscriber.push(callback);
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
    this.disconnect();
    this.socket = new WebSocket(this.url);
    this.socket.onopen = function() {
      if (ROS.onopen) ROS.onopen();
    };
    this.socket.onclose = function() {
      ROS.disconnect();
      if (ROS.onclose) ROS.onclose();
      setTimeout(function() { ROS.connect(); }, 500);
    };
    this.socket.onerror = function() {
      ROS.disconnect();
      if (ROS.onclose) ROS.onclose();
      setTimeout(function() { ROS.connect(); }, 500);
    };
    this.socket.onmessage = function(e) {
      var data = JSON.parse(e.data);
      if (data.msg && data.receiver in ROS.services && data.msg.id in ROS.services[data.receiver]) {
        var service = ROS.services[data.receiver];
        var id = data.msg.id;
        delete data.msg.id;
        if (service[id]) service[id](data.msg);
        delete service[id];
      } else if (data.receiver in ROS.subscribers) {
        ROS.subscribers[data.receiver].map(function(subscriber) {
          subscriber(data.msg);
        });
      } else if (ROS.onmessage) {
        ROS.onmessage(data);
      }
    };
    this.send = function(json) {
      var data = JSON.stringify(json);
      if (ROS.socket) {
        try {
          ROS.socket.send(data);
        } catch (e) {
          // INVALID_STATE_ERR: DOM Exception 11
        }
      }
    };
  }
};
