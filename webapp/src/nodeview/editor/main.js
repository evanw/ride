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

function loadSampleGraph(nodes) {
	nodes.push(new Node('gscam', [], ['image']));
	nodes.push(new Node('cmvision', ['image'], ['blobs']));
	nodes.push(new Node('ar_recog', ['image'], ['tags']));
	nodes.push(new Node('merge_blobs', ['blob_list'], ['blob_list']));
	nodes.push(new Node('filter_blobs', ['blob_list'], ['orange_over_green', 'green_over_orange', 'orange', 'green', 'yellow', 'pink']));
	nodes.push(new Node('seek_objects', ['blob_list'], ['twist']));
	nodes.push(new Node('avoid_objects', ['blob_list'], ['twist']));
	nodes.push(new Node('irobot_create_2_1', ['twist'], []));
	for (var i = 0; i < nodes.length; i++) {
		nodes[i].generateHTML();
	}

	function pos(index, x, y) {
		$(nodes[index].element).css({ left: x + 'px', top: y + 'px' });
		nodes[index].updateRects();
	}
	pos(0, 30, 80);
	pos(1, 230, 30);
	pos(2, 230, 130);
	pos(3, 430, 30);
	pos(4, 480, 130);
	pos(5, 730, 130);
	pos(6, 730, 230);
	pos(7, 930, 180);

	function connect(a, b, c, d) {
		nodes[a].outputs[b].inputs.push(nodes[c].inputs[d]);
		nodes[c].inputs[d].outputs.push(nodes[a].outputs[b]);
	}
	connect(0, 0, 1, 0);
	connect(0, 0, 2, 0);
	connect(1, 0, 3, 0);
	connect(3, 0, 4, 0);
	connect(4, 2, 5, 0);
	connect(4, 3, 6, 0);
	connect(5, 0, 7, 0);
	connect(6, 0, 7, 0);
}

$(window).load(function() {
	c = $('#canvas')[0].getContext('2d');
	editor = new Editor();

	// TODO: remove this, just for testing
	loadSampleGraph(editor.doc.nodes);

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

		if ((e.ctrlKey || e.metaKey) && e.which == 'A'.charCodeAt(0)) {
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
