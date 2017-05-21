( function() {
  'use strict';

  // Initiate a WebSocket connection
  var ws = new WebSocket( 'ws://' + window.location.host );

  // Set up the scene SVG element ASAP to avoid reflowing
  var width = 600;
  var height = width;
  var xscale = d3.scale.linear()
    .domain( [ 0, 100 ] )
    .range( [ 0, width ] );
  var yscale = d3.scale.linear()
    .domain( [ 0, 100 ] )
    .range( [ height, 0 ] );
  var ryscale = d3.scale.linear()
    .domain( [ 0, 100 ] )
    .range( [ 0, height ] );
  var scene = d3.select( '.scene' )
    .attr( 'width', width )
    .attr( 'height', height );

  var simTrack = [];
  var slamTrack = [];
  var simTrackSvg = scene.append( 'g' )
    .attr( 'class', 'simtrack' );
  var slamTrackSvg = scene.append( 'g' )
    .attr( 'class', 'slamtrack' );
  var lidarLines = scene.append( 'g' )
    .attr( 'class', 'lidar-lines' );
  var vehicle = scene.append( 'g' )
    .attr( 'class', 'vehicle' );

  // Since y increases upward in the simulation, but downward in SVG,
  // negate phi to get the correct CCW rotation.
  function transformEllipse( d ) {
    return 'translate(' + xscale( d.cx ) + ',' + yscale( d.cy ) + ') ' +
      'rotate(' + -d.phi * 180 / Math.PI + ')';
  }

  function xyTranslate( d ) {
    return 'translate(' + xscale( d.cx ) + ',' + yscale( d.cy ) + ') ';
  }

  function resetSim() {
    simTrack.length = 0;
    slamTrack.length = 0;
    simTrackSvg.selectAll( 'line' ).remove();
    slamTrackSvg.selectAll( 'line' ).remove();
    scene.selectAll( '.feature' ).remove();
    scene.selectAll( '.feature-ellipse' ).remove();
    scene.selectAll( '.lidar-sweep' ).remove();
    vehicle.selectAll( 'ellipse' ).remove();
    vehicle.selectAll( '.lidar-sweep' ).remove();
  }

  function sendReset() {
    ws.send( JSON.stringify( {
      type: 'request',
      text: 'reset',
      id: 2,
      date: Date.now()
    } ) );
  }

  function drawWaypoints( data ) {
    scene.selectAll( '.waypoint' )
      .data( data )
      .enter().append( 'circle' )
      .attr( 'r', 0 )
      .transition()
      .duration( 750 )
      .attr( 'r', 4 )
      .attr( 'cx', function( d ) {
        return xscale( d.x );
      } )
      .attr( 'cy', function( d ) {
        return yscale( d.y );
      } )
      .attr( 'class', 'waypoint' );
  }

  function drawLandmarks( data ) {

    var l = 10; // Edge length of square

    var landmarks = scene.selectAll( 'g .landmarks' )
      .data( data )
      .enter().append( 'g' )
      .attr( 'class', 'landmarks' )
      .attr( 'transform', function( d ) {
        return 'translate(' + ( xscale( d.x ) - l / 2 ) + ',' + ( yscale( d.y ) - l / 2 ) + ')';
      } );

    landmarks.append( 'rect' )
      .attr( 'width', 0 )
      .attr( 'height', 0 )
      .transition()
      .duration( 750 )
      .attr( 'width', l )
      .attr( 'height', l )
      .attr( 'rx', l / 10 )
      .attr( 'ry', l / 10 );
  }


  function drawSimTrack( data ) {

    simTrack.push( {
      'x': xscale( data.ideal.x ),
      'y': yscale( data.ideal.y )
    } );

    slamTrack.push( {
      'x': xscale( data.slam.x ),
      'y': yscale( data.slam.y )
    } );

    var n = simTrack.length;

    if ( n > 1 ) {
      simTrackSvg
        .append( 'line' )
        .attr( 'x1', simTrack[ n - 2 ].x )
        .attr( 'y1', simTrack[ n - 2 ].y )
        .attr( 'x2', simTrack[ n - 1 ].x )
        .attr( 'y2', simTrack[ n - 1 ].y );

      slamTrackSvg
        .append( 'line' )
        .attr( 'x1', slamTrack[ n - 2 ].x )
        .attr( 'y1', slamTrack[ n - 2 ].y )
        .attr( 'x2', slamTrack[ n - 1 ].x )
        .attr( 'y2', slamTrack[ n - 1 ].y );
    }
  }

  function drawLidar( data ) {
    var i = -1;
    while ( ++i < data.length ) {
      scene.selectAll( '.lidar-lines' )
        .append( 'line' )
        .attr( 'x1', xscale( data[ i ].x1 ) )
        .attr( 'y1', yscale( data[ i ].y1 ) )
        .attr( 'x2', xscale( data[ i ].x2 ) )
        .attr( 'y2', yscale( data[ i ].y2 ) )
        .transition()
        .duration( 250 )
        .style( 'opacity', 0 )
        .each( 'end', function() {
          d3.select( this )
            .remove();
        } );
    }
  }

  function drawVehicle( data ) {
    var nSigma = 2;

    var arc = d3.svg.arc()
      .innerRadius( 20 )
      .outerRadius( xscale( 30 ) )
      .startAngle( 0 )
      .endAngle( Math.PI );

    function arcTransform( d ) {
      return 'translate(' + xscale( d.cx ) + ',' + yscale( d.cy ) + ') ' +
        'rotate(' + -d.vehicle_phi * 180 / Math.PI + ')';
    }

    vehicle.selectAll( 'ellipse' )
      .data( data )
      .attr( 'rx', function( d ) {
        return xscale( nSigma * d.rx );
      } )
      .attr( 'ry', function( d ) {
        return ryscale( nSigma * d.ry );
      } )
      .attr( 'transform', transformEllipse )
      .enter().append( 'ellipse' )
      .attr( 'rx', function( d ) {
        return xscale( nSigma * d.rx );
      } )
      .attr( 'ry', function( d ) {
        return ryscale( nSigma * d.ry );
      } )
      .attr( 'transform', transformEllipse );

    vehicle.selectAll( '.lidar-sweep' )
      .data( data )
      .attr( 'd', arc )
      .attr( 'transform', arcTransform )
      .enter().append( 'path' )
      .attr( 'class', 'lidar-sweep' )
      .attr( 'd', arc )
      .attr( 'transform', arcTransform );
  }

  function drawFeatures( data ) {
    var nSigma = 2;

    function rx( d ) {
      return xscale( nSigma * d.rx );
    }

    function ry( d ) {
      return ryscale( nSigma * d.ry );
    }

    // Display feature symbols
    scene.selectAll( '.feature' )
      .data( data )
      .attr( 'transform', xyTranslate )
      .enter().append( 'path' )
      .attr( 'class', 'feature' )
      .attr( 'd', d3.svg.symbol().type( 'circle' ) )
      .attr( 'transform', xyTranslate );

    // Display feature uncertainty ellipses
    scene.selectAll( '.feature-ellipse' )
      .data( data )
      .attr( 'rx', rx )
      .attr( 'ry', ry )
      .attr( 'transform', transformEllipse )
      .enter().append( 'ellipse' )
      .attr( 'class', 'feature-ellipse' )
      .attr( 'rx', rx )
      .attr( 'ry', ry )
      .attr( 'transform', transformEllipse );

  }

  function drawVehicleParticles( data ) {
    scene.selectAll( '.vehicle-particle' )
      .data( data )
      .attr( 'cx', function( d ) {
        return xscale( d.x );
      } )
      .attr( 'cy', function( d ) {
        return yscale( d.y );
      } )
      .enter().append( 'circle' )
      .attr( 'r', 2 )
      .attr( 'cx', function( d ) {
        return xscale( d.x );
      } )
      .attr( 'cy', function( d ) {
        return yscale( d.y );
      } )
      .attr( 'class', 'vehicle-particle' );
  }


  // Websocket message dispatch table. Keys are message.type; values are
  // the matching draw callbacks.
  var dispatchTable = {
    'waypoints': drawWaypoints,
    'landmarks': drawLandmarks,
    'tracks': drawSimTrack,
    'lidar': drawLidar,
    'vehicle-ellipse': drawVehicle,
    'feature-ellipse': drawFeatures,
    'vehicle-particles': drawVehicleParticles
  };

  ws.onopen = function( event ) {
    try {
      ws.send( JSON.stringify( {
        type: 'update',
        text: 'ready',
        id: 0,
        date: Date.now()
      } ) );
      sendReset();
      resetSim();
    } catch ( e ) {
      console.log( e );
    }

    // For prototyping - start sim on page load (don't wait for button)
    ws.send( JSON.stringify( {
      type: 'request',
      text: 'start',
      id: 1,
      date: Date.now()
    } ) );
  }

  // Handler for messages received from server
  ws.onmessage = function( event ) {
    var msg = JSON.parse( event.data );

    // assert && assert( dispatchTable.hasOwnProperty( msg.type ),
    //   'No callback for msg.type = \'' + msg.type + '\'');

    if ( dispatchTable.hasOwnProperty( msg.type ) ) {
      dispatchTable[ msg.type ]( msg.data );
    }
  }

  d3.select( '#start' ).on( 'click', function() {
    ws.send( JSON.stringify( {
      type: 'request',
      text: 'start',
      id: 1,
      date: Date.now()
    } ) );
  } );

  d3.select( '#pause' ).on( 'click', function() {
    ws.send( JSON.stringify( {
      type: 'request',
      text: 'pause',
      id: 3,
      date: Date.now()
    } ) );
  } );

  d3.select( '#reset' ).on( 'click', function() {
    sendReset();
    resetSim();
  } );

} )();
