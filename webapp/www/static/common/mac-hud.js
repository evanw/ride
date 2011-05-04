function makeID() {
	return 'id' + Math.random().toString().substr(2);
}

function HUD(parentNode, title) {
	this.title = title;
	this.selectionIndex = 0;
	this.contents = [];
	this.buttons = [];
	this.element = document.createElement('div');
	this.element.className = 'mac-hud';
	parentNode.appendChild(this.element);
	this.updateHTML();

	var this_ = this;
	$(document.body).keydown(function(e) {
		if (e.which == 38 && this_.selectionIndex > 0) {
			this_.setSelectionIndex(this_.selectionIndex - 1);
		} else if (e.which == 40 && this_.selectionIndex < this_.contents.length - 1) {
			this_.setSelectionIndex(this_.selectionIndex + 1);
		} else if (e.which == 13) {
			this_.defaultAction();
		} else if (e.which == 27) {
			this_.hide();
		}
	});
}

HUD.prototype.defaultAction = function() {
	for (var i = 0; i < this.buttons.length; i++) {
		var button = this.buttons[i];
		if (button instanceof HUD.DefaultButton) {
			button.callback();
		}
	}
};

HUD.prototype.isVisible = function() {
	return $(this.element).is(':visible');
};

HUD.prototype.updateHTML = function() {
	var closeButtonID = makeID();
	var html = '';
	html += '<div class="topbar"></div>';
	html += '<div id="' + closeButtonID + '" class="closebutton">&times;</div>';
	html += '<div class="title">' + this.title + '</div>';
	html += '<div class="rowlist">';
	for (var i = 0; i < this.contents.length; i++) {
		var row = this.contents[i];
		if (row instanceof HUD.Row) {
			html += '<div id="' + row.id + '" class="row' + (i == this.selectionIndex ? ' selected' : '') + '">' + row.html + '</div>';
		} else if (row instanceof HUD.Box) {
			html += '<div id="' + row.id + '" class="box">' + row.html + '</div>';
		} else if (row instanceof HUD.Textbox) {
			html += '<div class="textbox">' + row.label + ': <input id="' + row.id + '" type="text" value="' + row.text + '" /></div>';
		}
	}
	html += '</div>';
	html += '<div class="buttonbar">';
	for (var i = 0; i < this.buttons.length; i++) {
		html += this.buttons[i].getHTML();
	}
	html += '</div>';
	this.element.innerHTML = html;

	// hook up events
	var this_ = this;
	for (var i = 0; i < this.contents.length; i++) {
		var row = this.contents[i];
		row.element = document.getElementById(row.id);
		if (row instanceof HUD.Row) {
			(function(i) {
				$(row.element).mousedown(function() {
					this_.setSelectionIndex(i);
				});
			})(i);
			$(row.element).dblclick(function() {
				this_.defaultAction();
			});
		} else if (row instanceof HUD.Textbox) {
			(function(row) {
				$(row.element).bind('input', function() {
					row.text = row.element.value;
					row.callback(row);
				});
			})(row);
		}
	}
	for (var i = 0; i < this.buttons.length; i++) {
		var button = this.buttons[i];
		button.element = document.getElementById(button.id);
		$(button.element).click(button.callback);
	}
	$('#' + closeButtonID).click(function() {
		this_.hide();
	});
};

HUD.prototype.setSelectionIndex = function(selectionIndex) {
	if (this.selectionIndex >= 0 && this.selectionIndex < this.contents.length) {
		var selected = this.contents[this.selectionIndex];
		if (selected instanceof HUD.Row) {
			document.getElementById(selected.id).className = 'row';
		}
	}
	this.selectionIndex = selectionIndex;
	if (this.selectionIndex >= 0 && this.selectionIndex < this.contents.length) {
		var selected = this.contents[this.selectionIndex];
		if (selected instanceof HUD.Row) {
			document.getElementById(selected.id).className = 'selected row';
		}
	}
};

HUD.prototype.setContents = function(contents) {
	this.contents = contents;
	this.updateHTML();
};

HUD.prototype.setButtons = function(buttons) {
	this.buttons = buttons;
	this.updateHTML();
};

HUD.prototype.show = function() {
	this.setSelectionIndex(0);
	$(this.element).stop().show().css({ opacity: 0 }).animate({ opacity: 1 });
};

HUD.prototype.hide = function() {
	$(this.element).stop().fadeOut();
};

HUD.Row = function(html) {
	this.html = html;
	this.element = null;
	this.id = makeID();
};

HUD.Box = function(html) {
	this.html = html;
	this.element = null;
	this.id = makeID();
};

HUD.Textbox = function(label, text, callback) {
	this.label = label;
	this.text = text;
	this.callback = callback;
	this.element = null;
	this.id = makeID();
};

HUD.DefaultButton = function(name, callback) {
	this.name = name;
	this.element = null;
	this.callback = callback;
	this.id = makeID();
};

HUD.DefaultButton.prototype.getHTML = function() {
	return '<div id="' + this.id + '" class="button">' + this.name + '</div>';
};

HUD.Button = function(name, callback) {
	this.name = name;
	this.element = null;
	this.callback = callback;
	this.id = makeID();
};

HUD.Button.prototype.getHTML = function() {
	return '<div id="' + this.id + '" class="button">' + this.name + '</div>';
};
