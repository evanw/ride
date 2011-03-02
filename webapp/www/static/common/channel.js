var Channel = (function() {
	////////////////////////////////////////////////////////////////////////////////
	// Container
	////////////////////////////////////////////////////////////////////////////////

	function Container() {
		this.map = {};
		this.callbacks = [];
	}
	
	Container.prototype.lookup = function(names, index) {
		index = index || 0;
		if (index >= names.length) {
			return this;
		}
		var name = names[index];
		if (!this.map.hasOwnProperty(name)) {
			this.map[name] = new Container();
		}
		return this.map[name].lookup(names, index + 1);
	};
	
	Container.prototype.subscribe = function(callback) {
		this.callbacks.push(callback);
	};
	
	Container.prototype.unsubscribe = function(callback) {
		for (var i = 0; i < this.callbacks.length; i++) {
			if (this.callbacks[i] == callback) {
				this.callbacks.splice(i--, 1);
			}
		}
	};

	Container.prototype.publish = function(data) {
		for (var i = 0; i < this.callbacks.length; i++) {
			this.callbacks[i](data);
		}
	};

	var root = new Container();
	var socket = null;

	////////////////////////////////////////////////////////////////////////////////
	// Channel
	////////////////////////////////////////////////////////////////////////////////

	function Channel(name) {
		this.name = name;
	}

	Channel.prototype.subscribe = function(callback) {
		root.lookup(this.name).subscribe(callback);
	};

	Channel.prototype.unsubscribe = function(callback) {
		root.lookup(this.name).unsubscribe(callback);
	};

	Channel.prototype.publish = function(data) {
		data.channel = this.name;
		if (socket) socket.send(JSON.stringify(data));
	};
	
	////////////////////////////////////////////////////////////////////////////////
	// socket.io
	////////////////////////////////////////////////////////////////////////////////

	document.write('<script type="text/javascript" src="http://' + document.location.hostname + ':5000/socket.io/socket.io.js"></script>');

	$(window).load(function() {
		socket = new io.Socket(document.location.hostname, { port: 5000 });
		socket.on('connect', function() {
			root.lookup(['server', 'status']).publish({ status: 'connected' });
		});
		socket.on('disconnect', function() {
			root.lookup(['server', 'status']).publish({ status: 'disconnected' });
		});
		socket.on('message', function(data) {
			var json = JSON.parse(data);
			root.lookup(json.channel).publish(json);
		});
		socket.connect();
	});

	return function() {
		return new Channel(Array.prototype.slice.call(arguments));
	};
})();
