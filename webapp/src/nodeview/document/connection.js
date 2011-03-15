function Connection(parent) {
	this.parent = parent;
	this.name = '';
	this.id = 0;
	this.connections = [];
	this.element = null;
	this.rect = null;
}

Connection.prototype.fromJSON = function(json) {
	this.name = json.name;
	this.id = json.id;
	this.connections = []; // this will be set later, after we have read all nodes
	return this;
};

Connection.prototype.toJSON = function() {
	return {
		name: this.name,
		connections: this.connections.map(function(c) {
			return {
				nodeID: c.parent.id,
				connectionID: c.id
			};
		})
	};
};

Connection.prototype.connectTo = function(other) {
	this.connections.addOnce(other);
	other.connections.addOnce(this);
};

Connection.prototype.disconnectFrom = function(other) {
	this.connections.removeAll(other);
	other.connections.removeAll(this);
};
