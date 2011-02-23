var nextNodeID = 0;

function Node(title, inputNames, outputNames) {
	this.id = nextNodeID++;
	this.title = title;
	this.inputs = [];
	this.outputs = [];
	this.rect = null;
	this.editRect = null;
	this.element = null;
	this.createElement();

	this.popup = new Popup().setDirection('right').setHTML(this.generatePopupHTML());

	for (var i = 0; i < inputNames.length; i++) {
		this.inputs.push(new Input(inputNames[i]));
	}
	for (var i = 0; i < outputNames.length; i++) {
		this.outputs.push(new Output(outputNames[i]));
	}
}

Node.prototype.generatePopupHTML = function() {
	// TODO: this will be different obviously, just for testing right now
	var html = '';
	html += '<table>';
	for (var name in this) {
		if (this.hasOwnProperty(name)) {
			html += '<tr><td>' + name + '</td><td><input type="text" value="' + textToHTML(this[name] + '') + '"></td></tr>';
		}
	}
	html += '</table>';
	return html;
};

Node.prototype.addInput = function(input) {
	this.inputs.push(input);
	input.parent = this;
};

Node.prototype.addOutput = function(output) {
	this.outputs.push(output);
	output.parent = this;
};

Node.prototype.createElement = function() {
	this.element = document.createElement('div');
	this.element.className = 'node';
	document.body.appendChild(this.element);
};

Node.prototype.deleteElement = function() {
	this.element.parentNode.removeChild(this.element);
	this.element = null;

	this.popup.deleteElement();
};

Node.prototype.updateRects = function() {
	this.rect = Rect.getFromElement(this.element, true);
	this.editRect = Rect.getFromElement($(this.element).find('.edit-link span')[0], false);

	for (var i = 0; i < this.inputs.length; i++) {
		var input = this.inputs[i];
		input.rect = Rect.getFromElement(input.element, false);
	}
	
	for (var i = 0; i < this.outputs.length; i++) {
		var output = this.outputs[i];
		output.rect = Rect.getFromElement(output.element, false);
	}

	this.popup.setAnchor(this.editRect.right + 3, this.editRect.centerY);
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
