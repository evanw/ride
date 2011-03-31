String.prototype.toHTML = function() {
	return this.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;').replace(/'/g, '&apos;');
};

Array.prototype.contains = function(element) {
	for (var i = 0; i < this.length; i++) {
		if (this[i] === element) {
			return true;
		}
	}
	return false;
};

Array.prototype.map = function(func) {
	var result = [];
	for (var i = 0; i < this.length; i++) {
		result.push(func(this[i]));
	}
	return result;
};

Array.prototype.addOnce = function(element) {
	for (var i = 0; i < this.length; i++) {
		if (this[i] === element) {
			return;
		}
	}
	this.push(element);
};

Array.prototype.removeAll = function(element) {
	for (var i = 0; i < this.length; i++) {
		if (this[i] === element) {
			this.splice(i--, 1);
		}
	}
};

Array.prototype.removeOnce = function(element) {
	for (var i = 0; i < this.length; i++) {
		if (this[i] === element) {
			this.splice(i, 1);
			return;
		}
	}
};

function drawLink(c, ax, ay, bx, by) {
	c.strokeStyle = 'yellow';
	c.fillStyle = 'yellow';
	c.lineWidth = 2;
	c.shadowBlur = 3;
	c.shadowColor = 'black';
	c.shadowOffsetY = 1;

	c.beginPath();
	c.moveTo(ax, ay);
	c.bezierCurveTo(ax + 100, ay, bx - 90, by, bx, by);
	c.stroke();

	// Draw arrow head
	var t = 0.95, invT = 1 - t;
	var t0 = invT * invT * invT;
	var t1 = invT * invT * t * 3;
	var t2 = invT * t * t * 3;
	var t3 = t * t * t;
	var x = t0 * ax + t1 * (ax + 100) + t2 * (bx - 100) + t3 * bx;
	var y = t0 * ay + t1 * ay + t2 * by + t3 * by;
	var angle = Math.atan2(by - y, bx - x);
	var sin = Math.sin(angle);
	var cos = Math.cos(angle);
	c.beginPath();
	c.moveTo(bx, by);
	c.lineTo(bx - 10 * cos - 5 * sin, by - 10 * sin + 5 * cos);
	c.lineTo(bx - 10 * cos + 5 * sin, by - 10 * sin - 5 * cos);
	c.fill();
}
