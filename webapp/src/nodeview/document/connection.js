function Connection(parent) {
	this.parent = parent;
	this.name = '';
	this.connections = [];
	this.element = null;
	this.rect = null;
}

Connection.prototype.fromJSON = function(json) {
	this.name = json.name;
	this.connections = []; // this will be set later, after we have read all nodes
	return this;
};

Connection.prototype.toJSON = function() {
	return {
		name: this.name
		// TODO: how to store connections?
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
