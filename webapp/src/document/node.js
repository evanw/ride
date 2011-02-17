var nextNodeID = 0;

function Node(title) {
	this.id = nextNodeID++;
	this.title = title;
	this.inputs = [];
	this.outputs = [];
	this.rect = null;
	this.element = null;
	this.createElement();
}

Node.prototype.getInputFromNode = function(node) {
	for (var i = 0; i < this.inputs.length; i++) {
		if (this.inputs[i].connectsToNode(node)) {
			return this.inputs[i];
		}
	}
	return null;
};

Node.prototype.getOutputToNode = function(node) {
	for (var i = 0; i < this.outputs.length; i++) {
		if (this.outputs[i].connectsToNode(node)) {
			return this.outputs[i];
		}
	}
	return null;
};

Node.prototype.createElement = function() {
	this.element = document.createElement('div');
	this.element.className = 'node';
	document.body.appendChild(this.element);
};

Node.prototype.updateRects = function() {
	this.rect = Rect.getFromElement(this.element, true);
	
	for (var i = 0; i < this.inputs.length; i++) {
		var input = this.inputs[i];
		input.rect = Rect.getFromElement(input.element, false);
	}
	
	for (var i = 0; i < this.outputs.length; i++) {
		var output = this.outputs[i];
		output.rect = Rect.getFromElement(output.element, false);
	}
};

Node.prototype.generateHTML = function() {
	var this_ = this;
	var html = '<table><tr><td class="title">' + this.title + '</td><td class="edit-link"><span>edit</span></td></tr></table>';
	html += '<table><tr><td>';

	// inputs to html
	for (var i = 0; i < this.inputs.length; i++) {
		html += '<div class="input"><div class="bullet" id="node' + this.id + '-input' + i + '">';
		html += '<div></div></div>&nbsp;' + textToHTML(this.inputs[i].name) + '</div>';
	}

	html += '</td><td>';

	// outputs to html
	for (var i = 0; i < this.outputs.length; i++) {
		html += '<div class="output">' + textToHTML(this.outputs[i].name) + '&nbsp;';
		html += '<div class="bullet" id="node' + this.id + '-output' + i + '"><div></div></div></div>';
	}

	// change html
	html += '</td></tr></table>';
	this.element.innerHTML = html;

	// reset all input elements
	for (var i = 0; i < this.inputs.length; i++) {
		this.inputs[i].element = document.getElementById('node' + this.id + '-input' + i);
	}

	// reset all output elements
	for (var i = 0; i < this.outputs.length; i++) {
		this.outputs[i].element = document.getElementById('node' + this.id + '-output' + i);
	}
	
	this.updateRects();
};
