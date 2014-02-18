var ros = new ROSLIB.Ros({
	url : 'ws://' + window.location.hostname + ':9090'
  });

var speechPub = new ROSLIB.Topic({
	ros : ros,
	name : '/recognized_command',
	messageType : 'pr2_pbd_speech_recognition/Command'
});

var guiPub = new ROSLIB.Topic({
	ros : ros,
	name : '/gui_command',
	messageType : 'pr2_pbd_interaction/GuiCommand'
});

var expListener = new ROSLIB.Topic({
	ros : ros,
	name : '/experiment_state',
	messageType : 'pr2_pbd_interaction/ExperimentState'
});

var expListenerSrvCli = new ROSLIB.Service({
	ros : ros,
	name : '/get_experiment_state',
	serviceType : 'pr2_pbd_interaction/GetExperimentState'
});

window.lockUpdate = false;

window.addEventListener("load", function() {
	[].slice.call(document.querySelectorAll("button[com]")).forEach(function(el) {
		el.addEventListener("click", function() {
			var relCom = new ROSLIB.Message({
				command : el.getAttribute("com")
			});
			speechPub.publish(relCom);
		});
	});

	var actsListCont = document.querySelector("#actsList");
	var addButSpan = document.querySelector("#addButs");
	var delButSpan = document.querySelector("#delButs");
	var stepsSpan = document.querySelector("#curAct");


	//code for drawing the current action and actions
	var drawState = function(state) {
		if (window.lockUpdate) 
			return;
		//draw action list
		actsListCont.innerHTML = "";
		state.action_names.forEach(function(act_n) {
			var dv = document.createElement("div");
			dv.innerHTML = act_n;
			dv.addEventListener("click", function() {
				speechPub.publish(new ROSLIB.Message({
					command: "switch-to-action " + act_n
				}));
			});
			actsListCont.appendChild(dv);
		});
		actsListCont.querySelectorAll("div")[state.selected_action].className = 
			"selected";

		//current action:
		addButSpan.innerHTML = "";
		delButSpan.innerHTML = "";
		stepsSpan.innerHTML = "";

		var actXML = new DOMParser().parseFromString(state.action_xml, "text/xml");
		var steps = [].slice.call(actXML.querySelectorAll(':root>actions>action'));
		//html for one step
		var dispStep = function(stepXml, i) {

			var outerCont = document.createElement("div");

			var sel = document.createElement("select");
			sel.innerHTML = "<option value='0'>Action</option>" + 
				"<option value='1'>Pose</option><option value='3'>Trajectory</option>";
			outerCont.appendChild(sel);

			//html for stuff besides +/-, and type buttons
			var genHtml = function(actType) {
				var stepCont = document.createElement("span");

				switch (actType) {
					case "0"://action
						sel.selectedIndex = 0;
						var actsSelect = document.createElement("select");
						actsSelect.innerHTML = state.action_names.reduce(function(c, nm) {
							return c + "<option value='" + nm + "'>" + nm + "</option>";
						}, "");
						actsSelect.selectedIndex = state.action_ids.indexOf(parseInt(stepXml.getAttribute("id")));
						stepCont.appendChild(actsSelect);

						var saveBut = document.createElement("button");
						saveBut.innerHTML = "save";
						stepCont.appendChild(saveBut);
						saveBut.addEventListener("click", function() {
							guiPub.publish(new ROSLIB.Message({
								command: "select-action-step",
								param: (i + 1)
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "delete-last-step"
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "add-action-step " + actsSelect.value
							}));
						});
						break;
					case "1"://pose
						sel.selectedIndex = 1;
						var recPose = document.createElement("button");
						recPose.innerHTML = "record";
						stepCont.appendChild(recPose);
						recPose.addEventListener("click", function() {
							guiPub.publish(new ROSLIB.Message({
								command: "select-action-step",
								param: (i + 1)
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "delete-last-step"
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "save-pose"
							}));
						});
						break;
					case "2"://gripper
						
						break;
					case "3"://trajectory
						sel.selectedIndex = 2;
						var recording = false;
						var recBut = document.createElement("button");
						recBut.innerHTML = "Record";
						stepCont.appendChild(recBut);
						recBut.addEventListener("click", function() {
							if (recording) {
								recBut.innerHTML = "Record";
								speechPub.publish(new ROSLIB.Message({
									command: "stop-recording-motion"
								}));
							} else {
								window.lockUpdate = true;
								recBut.innerHTML = "Stop";
								guiPub.publish(new ROSLIB.Message({
									command: "select-action-step",
									param: (i + 1)
								}));
								speechPub.publish(new ROSLIB.Message({
									command: "delete-last-step"
								}));
								speechPub.publish(new ROSLIB.Message({
									command: "start-recording-motion"
								}));
							}
							recording = !recording;
						});
						break;
				}
				return stepCont;
			};

			sel.addEventListener("change", function () {
				outerCont.querySelector("span").remove();
				outerCont.appendChild(genHtml(sel.value));
			})

			outerCont.appendChild(genHtml(stepXml.getAttribute("type")));

			return outerCont;
		};

		var addBut = document.createElement("button");
		addBut.innerHTML = "+";
		addButSpan.appendChild(addBut);
		addBut.addEventListener("click", function() {
			guiPub.publish(new ROSLIB.Message({
				command: "select-action-step",
				param: 0
			}));
			speechPub.publish(new ROSLIB.Message({
				command: "save-pose"
			}));
		});

		steps.forEach(function(sx, i) {
			var delBut = document.createElement("button");
			delBut.innerHTML = "-";
			delButSpan.appendChild(delBut);
			delBut.addEventListener("click", function() {
				guiPub.publish(new ROSLIB.Message({
					command: "select-action-step",
					param: (i + 1)
				}));
				speechPub.publish(new ROSLIB.Message({
					command: "delete-last-step"
				}));
			});

			var addBut = document.createElement("button");
			addBut.innerHTML = "+";
			addButSpan.appendChild(addBut);
			addBut.addEventListener("click", function() {
				guiPub.publish(new ROSLIB.Message({
					command: "select-action-step",
					param: (i + 1)
				}));
				speechPub.publish(new ROSLIB.Message({
					command: "save-pose"
				}));
			});
			stepsSpan.appendChild(dispStep(sx, i));
		});
	};

	expListener.subscribe(function(state) {
		drawState(state);
	});


	expListenerSrvCli.callService(new ROSLIB.ServiceRequest({}), function(result) {
		drawState(result.state);
	});
});