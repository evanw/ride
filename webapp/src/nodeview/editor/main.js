var c;
var editor;

function resizeCanvas() {
	var minSize = editor.getMinSize();
	c.canvas.width = minSize.width;
	c.canvas.height = minSize.height;
}

function draw() {
	resizeCanvas();
	editor.draw(c);
}

$(window).load(function() {
	c = $('#canvas')[0].getContext('2d');
	editor = new Editor();

	draw();

	// need to preventDefault() here instead of mousedown because we
	// still want mousedown to move keyboard focus into the iframe
	$(document).bind('selectstart', function(e) {
		e.preventDefault();
	});

	$(document).mousedown(function(e) {
		editor.mousePressed(e.pageX, e.pageY);
		draw();
	});

	$(document).mousemove(function(e) {
		editor.mouseMoved(e.pageX, e.pageY);
		e.preventDefault();
		draw();
	});

	$(document).mouseup(function(e) {
		editor.mouseReleased(e.pageX, e.pageY);
		e.preventDefault();
		draw();
	});

	var focusCount = 0;
	$('input').live('focus', function() { focusCount++; });
	$('input').live('blur', function() { focusCount--; });

	$(document).keydown(function(e) {
		// disable keyboard shortcuts inside input elements
		if (focusCount > 0) {
			return;
		}

		if ((e.ctrlKey || e.metaKey) && e.which == 'Z'.charCodeAt(0)) {
			if (e.shiftKey) editor.redo();
			else editor.undo();
		} else if ((e.ctrlKey || e.metaKey) && e.which == 'A'.charCodeAt(0)) {
			editor.selectAll();
			e.preventDefault();
			draw();
		} else if (e.which == '\b'.charCodeAt(0)) {
			editor.deleteSelection();
			e.preventDefault();
			draw();
		}
	});

	$(window).resize(draw);
});
