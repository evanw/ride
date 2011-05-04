function Node() {
	this.x = 0;
	this.y = 0;
	this.id = 0;
	this.name = '';
	this.inputs = [];
	this.outputs = [];
	this.rect = null;
	this.editRect = null;
	this.element = null;
	this.extras = {};
}

Node.prototype.fromJSON = function(json) {
	this.x = json.x;
	this.y = json.y;
	this.id = json.id;
	this.name = json.name;
	this.inputs = json.inputs.map(function(i) {
		return new Connection(this).fromJSON(i);
	});
	this.outputs = json.outputs.map(function(i) {
		return new Connection(this).fromJSON(i);
	});
	this.extras = {};
	for (var x in json) {
		if (!(x in this)) {
			this.extras[x] = json[x];
		}
	}
	return this;
};

Node.prototype.toJSON = function() {
	var json = {
		x: this.x,
		y: this.y,
		id: this.id,
		name: this.name,
		inputs: this.inputs.map(function(i) {
			return i.toJSON();
		}),
		outputs: this.outputs.map(function(o) {
			return o.toJSON();
		})
	};
	for (var x in this.extras) {
		json[x] = this.extras[x];
	}
	return json;
};

Node.prototype.update = function(name, value) {
	if (this[name] !== value) {
		this[name] = value;
		this.generateHTML();
		this.updateRects();
		this.hidePopup();
	}
};

Node.prototype.generatePopupHTML = function() {
	var properties = [ 'name', 'pkg', 'exec' ];
	var labels = [ 'Name', 'Package', 'Executable' ];
	var html = '';
	html += '<table>';
	for (var i = 0; i < properties.length; i++) {
		var name = properties[i];
		var id = 'node' + this.id + '-prop' + i;
		html += '<tr><td>' + labels[i] + ':</td><td><input type="text" id="' + id + '" value="' + ((name in this ? this[name] : this.extras[name]) + '').toHTML() + '"></td></tr>';
	}
	html += '</table>';
	return html;
};

Node.prototype.bindPopupCallbacks = function() {
	// var this_ = this;
	// $('#node' + this.id + '-prop0').bind('input', function(e) {
	// 	editor.doc.updateNode(this_, 'name', e.target.value);
	// });
	// $('#node' + this.id + '-prop1').bind('input', function(e) {
	// 	editor.doc.updateNode(this_, 'pkg', e.target.value);
	// });
	// $('#node' + this.id + '-prop2').bind('input', function(e) {
	// 	editor.doc.updateNode(this_, 'exec', e.target.value);
	// });
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
	this.editRect = new Rect(0, 0, 0, 0);//Rect.getFromElement($(this.element).find('.edit-link span')[0], false);

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
	var html = '<table><tr><td class="title">' + this.name + '</td>' + /*'<td class="edit-link"><span>edit</span></td>' +*/ '</tr></table>';
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
	this.bindPopupCallbacks();
};

Node.prototype.hidePopup = function() {
	this.popup.hide();
};
