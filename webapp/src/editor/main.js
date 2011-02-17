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

	var nodes = editor.doc.nodes;
	nodes.push(new Node('foo'));
	nodes.push(new Node('bar_akjshdjkhajksdjkahsjdkhajkshjhajkshdjkahsd'));
	nodes.push(new Node('baz'));
	nodes[0].outputs.push(new Output('to_bar'));
	nodes[0].outputs.push(new Output('asd'));
	nodes[0].outputs.push(new Output('sdfssd'));
	nodes[0].outputs.push(new Output('eujhe'));
	nodes[1].inputs.push(new Input('from_foo'));
	nodes[1].outputs.push(new Output('to_baz'));
	nodes[2].outputs.push(new Output('blahsdl_sakdjhfk_sdkfja_ksdf'));
	nodes[2].inputs.push(new Input('from_bar'));
	nodes[0].outputs[0].nodes.push(nodes[1]);
	nodes[1].inputs[0].nodes.push(nodes[0]);
	nodes[1].outputs[0].nodes.push(nodes[2]);
	nodes[2].inputs[0].nodes.push(nodes[1]);
	nodes[0].generateHTML();
	nodes[1].generateHTML();
	nodes[2].generateHTML();

	draw();

	$(document).mousedown(function(e) {
		editor.mousePressed(e.pageX, e.pageY);
		e.preventDefault();
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

	$(document).keydown(function(e) {
		if ((e.ctrlKey || e.metaKey) && e.which == 'A'.charCodeAt(0)) {
			editor.selectAll();
			e.preventDefault();
			draw();
		}
	});

	$(window).resize(draw);
});
