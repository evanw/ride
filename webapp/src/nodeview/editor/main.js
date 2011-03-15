$(window).load(function() {
	var context = $('#canvas')[0].getContext('2d');
	var editor = new Editor(context);

	// need to preventDefault() here instead of mousedown because we
	// still want mousedown to move keyboard focus into the iframe
	$(document).bind('selectstart', function(e) {
		e.preventDefault();
	});

	$(document).mousedown(function(e) {
		editor.mousePressed(e.pageX, e.pageY);
	});

	$(document).mousemove(function(e) {
		editor.mouseMoved(e.pageX, e.pageY);
		e.preventDefault();
	});

	$(document).mouseup(function(e) {
		editor.mouseReleased(e.pageX, e.pageY);
		e.preventDefault();
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
		} else if (e.which == '\b'.charCodeAt(0)) {
			editor.deleteSelection();
			e.preventDefault();
		}
	});

	$(window).resize(function() {
		editor.draw();
	});
});
