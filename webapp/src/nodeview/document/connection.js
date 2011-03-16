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
	this._json_ids = json.connections; // will be remapped from list of ids to list of Connection objects after this
	this.connections = [];
	return this;
};

Connection.prototype.toJSON = function() {
	return {
		id: this.id,
		name: this.name,
		connections: this.connections.map(function(c) {
			return c.id;
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
