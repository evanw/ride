function Popup(cornerRadius, parentNode) {
	this.anchorX = 0;
	this.anchorY = 0;
	this.isVisible = false;
	this.direction = 'down';
	this.cornerRadius = cornerRadius || 10;

	this.element = document.createElement('div');
	this.element.className = 'mac-popup';
	(parentNode || document.body).appendChild(this.element);

	this.canvas = document.createElement('canvas');
	this.element.appendChild(this.canvas);

	this.content = document.createElement('div');
	this.element.appendChild(this.content);
}

Popup.prototype.setHTML = function(html) {
	this.content.innerHTML = html;
	this._update();
	return this;
};

Popup.prototype.setAnchor = function(x, y) {
	this.anchorX = x;
	this.anchorY = y;
	this._update();
	return this;
};

Popup.prototype.setDirection = function(direction) {
	this.direction = direction;
	this._update();
	return this;
};

Popup.prototype.show = function() {
	if (!this.isVisible) {
		this.isVisible = true;
		$(this.element).fadeIn('fast');
		this._update();
	}
};

Popup.prototype.hide = function() {
	if (this.isVisible) {
		this.isVisible = false;
		$(this.element).fadeOut('fast');
	}
};

Popup.prototype.deleteElement = function() {
	this.element.parentNode.removeChild(this.element);
	this.element = null;
};

Popup.prototype._update = function() {
	if (!this.isVisible) {
		return;
	}

	var c = this.canvas.getContext('2d');
	var r = this.cornerRadius;

	// temporarily expand this.element to accurately measure the size of this.content (otherwise it would word wrap)
	// TODO: find a better way of doing this
	this.element.style.width = '1000px';
	this.element.style.height = '1000px';

	var w = $(this.content).outerWidth() + 2 * r;
	var h = $(this.content).outerHeight() + 2 * r;
	var padding = r + 5;
	var x, y;

	switch (this.direction) {
	case 'left':
		x = w + this.cornerRadius;
		y = h / 2;
		break;

	case 'right':
		x = -this.cornerRadius;
		y = h / 2;
		break;

	case 'down':
		x = w / 2;
		y = -this.cornerRadius;
		break;

	case 'up':
		x = w / 2;
		y = h + this.cornerRadius;
		break;
	} 

	var minArrowSpace = 5;
	if (this.direction == 'up' || this.direction == 'down') {
		var windowWidth = window.innerWidth;
		var scrollX = document.body.scrollLeft;
		x = Math.min(x, Math.max(2 * r + minArrowSpace, this.anchorX - scrollX - padding));
		x = Math.max(x, Math.min(w - 2 * r - minArrowSpace, this.anchorX - scrollX + w - windowWidth + padding));
	} else if (this.direction == 'left' || this.direction == 'right') {
		var windowHeight = window.innerHeight;
		var scrollY = document.body.scrollTop;
		y = Math.min(y, Math.max(2 * r + minArrowSpace, this.anchorY - scrollY - padding));
		y = Math.max(y, Math.min(h - 2 * r - minArrowSpace, this.anchorY - scrollY + h - windowHeight + padding));
	}

	this.element.style.left = this.anchorX - x - padding + 'px';
	this.element.style.top = this.anchorY - y - padding + 'px';
	this.element.style.top = this.anchorY - y - padding + 'px';
	this.content.style.left = padding + r + 'px';
	this.content.style.top = padding + r + 'px';
	this.canvas.width = w + 2 * padding;
	this.canvas.height = h + 2 * padding;
	this.element.style.width = this.canvas.width + 'px';
	this.element.style.height = this.canvas.height + 'px';
	c.translate(padding, padding);

	function drawPath() {
		c.beginPath();
		c.arc(r, r, r, Math.PI, Math.PI * 3 / 2, false);
		if (this.direction == 'down') {
			c.lineTo(x - r, 0);
			c.lineTo(x, -r);
			c.lineTo(x + r, 0);
		}
		c.arc(w - r, r, r, Math.PI * 3 / 2, Math.PI * 2, false);
		if (this.direction == 'left') {
			c.lineTo(w, y - r);
			c.lineTo(w + r, y);
			c.lineTo(w, y + r);
		}
		c.arc(w - r, h - r, r, 0, Math.PI / 2, false);
		if (this.direction == 'up') {
			c.lineTo(x + r, h);
			c.lineTo(x, h + r);
			c.lineTo(x - r, h);
		}
		c.arc(r, h - r, r, Math.PI / 2, Math.PI, false);
		if (this.direction == 'right') {
			c.lineTo(0, y + r);
			c.lineTo(-r, y);
			c.lineTo(0, y - r);
		}
		c.closePath();
	}

	c.strokeStyle = 'rgba(0, 0, 0, 0.5)';
	drawPath.call(this);
	c.stroke();

	c.fillStyle = '#DFDFDF';
	c.shadowBlur = 4;
	c.shadowColor = 'rgba(0, 0, 0, 0.5)';
	c.shadowOffsetY = 2;
	drawPath.call(this);
	c.fill();
};
