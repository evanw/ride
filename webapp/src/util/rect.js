function Rect(left, top, width, height) {
	this.left = left;
	this.top = top;
	this.width = width;
	this.height = height;
	this.right = left + width;
	this.bottom = top + height;
	this.centerX = left + width / 2;
	this.centerY = top + height / 2;
}

Rect.getFromElement = function(element, noMargin) {
	var e = $(element);
	var offset = e.offset();
	return new Rect(
		offset.left - noMargin * parseInt(e.css('marginLeft'), 10),
		offset.top - noMargin * parseInt(e.css('marginTop'), 10),
		e.innerWidth(),
		e.innerHeight()
	);
};

Rect.prototype.contains = function(x, y) {
	return x >= this.left && x < this.right && y >= this.top && y < this.bottom;
};

Rect.prototype.intersects = function(rect) {
	return this.right > rect.left && rect.right > this.left && this.bottom > rect.top && rect.bottom > this.top;
};
