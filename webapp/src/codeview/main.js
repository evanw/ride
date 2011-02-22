$(window).load(function() {
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('Save', '/static/images/save.png')
	];

	var toolbar = new Toolbar();
	toolbar.setContents(contents);

	function resize() {
		$('#editor').css({ top: toolbar.height() + 'px' });
	}
	$(window).resize(resize);
	resize();

    var editor = ace.edit('editor');
    editor.setTheme('ace/theme/twilight');
	editor.renderer.setShowPrintMargin(false);
	editor.renderer.setHScrollBarAlwaysVisible(false);
	editor.setHighlightActiveLine(false);

    var PythonMode = require('ace/mode/python').Mode;
    editor.getSession().setMode(new PythonMode());
});
