////////////////////////////////////////////////////////////////////////////////

function AddNodeCommand(doc, node) {
	this.doc = doc;
	this.node = node;
}

AddNodeCommand.prototype.undo = function() {
	this.doc.removeNode(this.node);
};

AddNodeCommand.prototype.redo = function() {
	this.doc.addNode(this.node);
};

AddNodeCommand.prototype.mergeWith = function(command) {
	return false;
};

////////////////////////////////////////////////////////////////////////////////

function RemoveNodeCommand(doc, node) {
	this.doc = doc;
	this.node = node;
}

RemoveNodeCommand.prototype.undo = function() {
	this.doc.addNode(this.node);
};

RemoveNodeCommand.prototype.redo = function() {
	this.doc.removeNode(this.node);
};

RemoveNodeCommand.prototype.mergeWith = function(command) {
	return false;
};

////////////////////////////////////////////////////////////////////////////////

function UpdateNodeCommand(doc, node, name, value) {
	this.doc = doc;
	this.node = node;
	this.name = name;
	this.oldValue = node[name];
	this.newValue = value;
}

UpdateNodeCommand.prototype.undo = function() {
	this.doc.updateNode(this.node, this.name, this.oldValue);
	this.node[this.name] = this.oldValue;
};

UpdateNodeCommand.prototype.redo = function() {
	this.doc.updateNode(this.node, this.name, this.newValue);
};

UpdateNodeCommand.prototype.mergeWith = function(command) {
	if (command instanceof UpdateNodeCommand && this.name === command.name && this.newValue === command.oldValue) {
		this.newValue = command.newValue;
		return true;
	}
	return false;
};

////////////////////////////////////////////////////////////////////////////////

function SetSelectionCommand(doc, sel) {
	this.doc = doc;
	this.oldSel = doc.sel;
	this.newSel = sel;
}

SetSelectionCommand.prototype.undo = function() {
	this.doc.setSelection(this.oldSel);
};

SetSelectionCommand.prototype.redo = function() {
	this.doc.setSelection(this.newSel);
};

SetSelectionCommand.prototype.mergeWith = function(command) {
	if (command instanceof SetSelectionCommand) {
		this.newSel = command.newSel;
		return true;
	}
	return false;
};
