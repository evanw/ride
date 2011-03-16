window.channel = (function() {
	var map = {};
	var socket = null;

	function lookup(names) {
		var name = JSON.stringify(names);
		if (!map.hasOwnProperty(name)) {
			map[name] = new Channel();
		}
		return map[name];
	}

	////////////////////////////////////////////////////////////////////////////////
	// Channel
	// 
	// A permanent object that is created once for each channel and stored in map,
	// which is a map of channel lists (as JSON strings) to Channel objects. This
	// mapping is automatically done by the lookup() function above.
	////////////////////////////////////////////////////////////////////////////////

	function Channel() {
		this.callbacks = [];
		this.disabled = false;
	}
	
	Channel.prototype.subscribe = function(callback) {
		this.callbacks.push(callback);
	};
	
	Channel.prototype.unsubscribe = function(callback) {
		for (var i = 0; i < this.callbacks.length; i++) {
			if (this.callbacks[i] == callback) {
				this.callbacks.splice(i--, 1);
			}
		}
	};

	Channel.prototype.publish = function(data) {
		for (var i = 0; i < this.callbacks.length; i++) {
			this.callbacks[i](data);
		}
	};

	////////////////////////////////////////////////////////////////////////////////
	// Helper
	// 
	// A temporary object that exists to wrap the permanent channel objects above.
	// This is the object returned by the public channel() function.
	////////////////////////////////////////////////////////////////////////////////

	function Helper(names) {
		this.names = names;
	}

	Helper.prototype.subscribe = function(callback) {
		lookup(this.names).subscribe(callback);
	};

	Helper.prototype.unsubscribe = function(callback) {
		lookup(this.names).unsubscribe(callback);
	};

	Helper.prototype.publish = function(data) {
		// Is publishing disabled?
		if (lookup(this.names).disabled) return;
		
		// Send to other subscribers in this browser window
		lookup(this.names).publish(data);
		
		// Send to the server
		if (socket) {
			socket.send(JSON.stringify({
				'channel': this.names,
				'data': data
			}));
		}
	};
	
	Helper.prototype.disable = function() {
		lookup(this.names).disabled = true;
	};

	Helper.prototype.enable = function() {
		lookup(this.names).disabled = false;
	};
	
	////////////////////////////////////////////////////////////////////////////////
	// socket.io
	// 
	// This keeps one connection to the server open at all times, which is
	// automatically connected when the page loads.
	// 
	// TODO: thorough testing with dropped connections
	////////////////////////////////////////////////////////////////////////////////

	var port = 5000;
	document.write('<script type="text/javascript" src="http://' + document.location.hostname + ':' + port + '/socket.io/socket.io.js"></script>');

	$(window).load(function() {
		window.WEB_SOCKET_SWF_LOCATION = 'http://' + document.location.hostname + ':' + port + window.WEB_SOCKET_SWF_LOCATION;

		socket = new io.Socket(document.location.hostname, { 'port': port });
		socket.on('connect', function() {
			lookup(['server', 'status']).publish({ 'status': 'connected' });
		});
		socket.on('disconnect', function() {
			lookup(['server', 'status']).publish({ 'status': 'disconnected' });
		});
		socket.on('message', function(data) {
			var json = JSON.parse(data);
			lookup(json['channel']).publish(json['data']);
		});
		socket.connect();
	});

	return function() {
		return new Helper(Array.prototype.slice.call(arguments));
	};
})();
