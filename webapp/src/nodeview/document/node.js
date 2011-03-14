function Node() {
	this.x = 0;
	this.y = 0;
	this.id = 0;
	this.title = '';
	this.inputs = [];
	this.outputs = [];
	this.rect = null;
	this.editRect = null;
	this.element = null;
}

Node.prototype.fromJSON = function(json) {
	this.x = json.x;
	this.y = json.y;
	this.id = json.id;
	this.title = json.title;
	this.inputs = json.inputs.map(function(i) {
		return new Connection(this).fromJSON(i);
	});
	this.outputs = json.outputs.map(function(i) {
		return new Connection(this).fromJSON(i);
	});
	return this;
};

Node.prototype.toJSON = function() {
	return {
		x: this.x,
		y: this.y,
		id: this.id,
		title: this.title,
		inputs: this.inputs.map(function(i) {
			return i.toJSON();
		}),
		outputs: this.outputs.map(function(o) {
			return o.toJSON();
		})
	};
};

Node.prototype.update = function(name, value) {
	this[name] = value;
	this.generateHTML();
	this.updateRects();
	this.hidePopup();
};

Node.prototype.generatePopupHTML = function() {
	// TODO: this will be different obviously, just for testing right now
	var html = '';
	html += '<table>';
	for (var name in this) {
		if (this.hasOwnProperty(name)) {
			html += '<tr><td>' + name + '</td><td><input type="text" value="' + (this[name] + '').toHTML() + '"></td></tr>';
		}
	}
	html += '</table>';
	return html;
};

Node.prototype.createElement = function() {
	this.element = document.createElement('div');
	this.element.className = 'node';
	document.body.appendChild(this.element);

	this.popup = new Popup().setDirection('right');
	this.generateHTML();
	this.updateRects();
};

Node.prototype.deleteElement = function() {
	this.element.parentNode.removeChild(this.element);
	this.element = null;

	this.popup.deleteElement();
	this.popup = null;
};

Node.prototype.updateRects = function() {
	this.element.style.left = this.x + 'px';
	this.element.style.top = this.y + 'px';

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
		html += '<div></div></div>&nbsp;' + this.inputs[i].name.toHTML() + '</div>';
	}

	html += '</td><td>';

	// outputs to html
	for (var i = 0; i < this.outputs.length; i++) {
		html += '<div class="output">' + this.outputs[i].name.toHTML() + '&nbsp;';
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

Node.prototype.showPopup = function() {
	this.popup.setHTML(this.generatePopupHTML());
	this.popup.show();
};

Node.prototype.hidePopup = function() {
	this.popup.hide();
};
