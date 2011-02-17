function Output(name) {
	this.name = name;
	this.parent = null;
	this.inputs = [];
	this.element = null;
	this.rect = null;
}

Output.prototype.connectTo = function(input) {
	addOnce(this.inputs, input);
	addOnce(input.outputs, this);
};

Output.prototype.disconnectFrom = function(input) {
	removeAll(this.inputs, input);
	removeAll(input.outputs, this);
};
